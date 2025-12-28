#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"
#include <algorithm>

void intake_and_pushback(double angle, int intake_time, int push_time)
{

  // 转至180
  ChassisControlSTS.Over_count = 10;
  ChassisControlSTS.TurnTo(10, NormsAngle_0_360(angle), false);
  ChassisControlSTS.Over_count = 1;

  // 吸导入
  ToolMotor1.spin(forward, -16, voltageUnits::volt);
  ToolMotor2.spin(forward, 15, pct); // 递球
  ToolMotor3.spin(forward, 3, voltageUnits::volt);
  ChassisControlSTS.Move(5, 900, angle, 'n', true); // 1550

  // 使用距离传感器判断是否到达恒力区域 250是即将进入导入区的距离
  while (GetDistance_front() > 250)
  {
    wait(10, msec);
  }
  ChassisControlSTS.SetStopModCoast(); // 需要保留，否则容易插不进去
  ChassisControlSTS.StopMoving();
  wait(100, msec);

  // 恒力吸导入  pct可以保证匀速插入导入筒
  for (int i = 0; i < 200; i++)
  {
    ChassisControlSTS.ChassisLeftControl_pct(2);
    ChassisControlSTS.ChassisRightControl_pct(2);
    if (GetDistance_front() < 276)
    {
      break;
    }
    wait(10, msec);
  }

  // 轻微的蠕动
  for (int i = 0; i < intake_time; i++)
  {
    if (GetDistance_front() > 170)
    {
      ChassisControlSTS.ChassisLeftControl(1.5);
      ChassisControlSTS.ChassisRightControl(1.5);
    }
    else
    {
      ChassisControlSTS.ChassisLeftControl(0);
      ChassisControlSTS.ChassisRightControl(0);
    }
    wait(10, msec);
  }

  // 倒车 打球
  ChassisControlSTS.Move(7, 1500, angle, 'u', true); // 1550
  ToolMotor2.spin(forward, 20, pct);                 // 递球
  ToolMotor3.spin(forward, 3, voltageUnits::volt);

  // 提前打球
  ChassisControlSTS.WaitPositionArrive(1100);

  ToolMotor2.spin(forward, 16, voltageUnits::volt);
  ToolMotor3.spin(forward, 16, voltageUnits::volt);
  ChassisControlSTS.SetStopModCoast();
  ChassisControlSTS.StopMoving();
  // 努力进入对位
  ChassisControlSTS.ChassisLeftControl(-0.5);
  ChassisControlSTS.ChassisRightControl(-0.5);
  wait(push_time, msec);
  ChassisControlSTS.SetStopModHold();
  ToolMotor2.spin(forward, 0, voltageUnits::volt);
  ToolMotor3.spin(forward, 0, voltageUnits::volt);
  digitalG.set(0);
}

void intake_and_pushback_new(double angle, int intake_time, int push_time)
{
  // 吸导入
  SimulatedRemoteControl("R1");

  ChassisControlSTS.Move(5, 900, angle, 'n', true); // 1550

  // 使用距离传感器判断是否到达恒力区域
  while (GetDistance_front() > 372)
  {
    wait(10, msec);
  }
  ChassisControlSTS.SetStopModCoast(); // 需要保留，否则容易插不进去
  ChassisControlSTS.StopMoving();
  wait(100, msec);

  // AutoIntake();

  // 轻微的蠕动
  for (int i = 0; i < 200; i++)
  {
    ChassisControlSTS.ChassisLeftControl(5);
    ChassisControlSTS.ChassisRightControl(5);

    // 计算真正吸导入的时间
    if (GetDistance_front() <= 200)
    {
      intake_time -= 10;
    }

    // 时间到，退出
    if (intake_time <= 0)
    {
      break;
    }

    // 异色球，退出
    if ((stance == 'r' && OpticalHaveBlue(Optical1)) || (stance == 'b' && OpticalHaveRed(Optical1)))
    {
      break;
    }

    wait(10, msec);
  }

  // StopAutoIntake();

  // 倒车 打球
  ChassisControlSTS.Move(7, 1500, angle, 'u', true); // 1550
  ChassisControlSTS.WaitPositionArrive(500);

  // 提前打球
  ChassisControlSTS.WaitPositionArrive(1200);
  digitalH.set(0);

  SimulatedRemoteControl("L1");

  ChassisControlSTS.SetStopModCoast();
  ChassisControlSTS.StopMoving();
  // 努力进入对位
  ChassisControlSTS.ChassisLeftControl(-2);
  ChassisControlSTS.ChassisRightControl(-2);

  double push_start_time = GetNowTime();
  wait(500, msec);
  ChassisControlSTS.AutoPPSTurnTo(10, angle);
  double turn_end_time = GetNowTime();

  wait(std::max(0.0, push_time - (turn_end_time - push_start_time) * 1000), msec);
}

// 钩子
void Hook(double angle)
{
  ChassisControlSTS.Move_PPS = {0.0182, 0.5, 2};
  ChassisControlSTS.Move(16, 400, 'n'); // 1550

  ChassisControlSTS.AutoPPSTurnTo(10, angle - 30);
  ChassisControlSTS.CurveMove(10, 700, 800, 'd', true);
  ChassisControlSTS.WaitPositionArrive(700);
  ChassisControlSTS.StopMoving();

  ChassisControlSTS.Over_count = 5;
  ChassisControlSTS.AutoPPSTurnTo(2.3, angle);
  SimulatedRemoteControl("Stop");
  wait(200, msec);
  ChassisControlSTS.ChassisLeftControl_pct(-5);
  ChassisControlSTS.ChassisRightControl_pct(-5);
  wait(1000, msec);

  // 急刹车
  ChassisControlSTS.ChassisLeftControl(1);
  ChassisControlSTS.ChassisRightControl(1);
  wait(500, msec);
  ChassisControlSTS.ChassisLeftControl_pct(0);
  ChassisControlSTS.ChassisRightControl_pct(0);
  wait(400000, msec);
}

// 等待前方距离传感器到达某个时间，然后停止运动
void WaitFrontDistanceArriveThenStopMove(double speed, double val, double angle, double Distance)
{
  ChassisControlSTS.Move(speed, val, angle, 'n', true);
  // 等待到达指定距离
  while (GetDistance_front() > Distance)
  {
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
}

bool AutoIntakeFlag = true;

int fun()
{
  AutoIntakeFlag = true;
  // 分球暂停计数器
  int ErrorBall1 = 0;
  int ErrorBall2 = 0;
  while (AutoIntakeFlag)
  {
    // 分球
    if ((stance == 'r' && OpticalHaveBlue(Optical1)) || (stance == 'b' && OpticalHaveRed(Optical1)))
    {
      ErrorBall1 = 20;
    }
    else if ((stance == 'b' && OpticalHaveBlue(Optical1)) || (stance == 'r' && OpticalHaveRed(Optical1)))
    {
      ErrorBall1 = 0;
    }
    else
    {
      ErrorBall1 = -1;
    }

    if ((stance == 'r' && OpticalHaveBlue(Optical2)) || (stance == 'b' && OpticalHaveRed(Optical2)))
    {
      ErrorBall2 = 20;
    }
    else if ((stance == 'b' && OpticalHaveBlue(Optical2)) || (stance == 'r' && OpticalHaveRed(Optical2)))
    {
      ErrorBall2 = 0;
    }
    else
    {
      ErrorBall2 = -1;
    }
    //  分球逻辑
    // ErrorBall1： 靠前方的颜色传感器
    // ErrorBall2： 靠后方的颜色传感器
    // 0   ：正确颜色的球
    // -1  ：没有球
    // 1~20：错误的球，检测到错球值为20

    if (ErrorBall1 > 0 && ErrorBall2 > 0)
    {
      // 前后都是异色球
      ToolMotor2.spin(forward, 16, voltageUnits::volt);
    }
    else if (ErrorBall1 > 0 && ErrorBall2 < 0)
    {
      // 前异色，后没球
      ToolMotor2.spin(forward, 16, voltageUnits::volt);
    }
    else if (ErrorBall1 > 0 && ErrorBall2 == 0)
    {
      // 前异色，后同色
      ToolMotor2.spin(forward, -16, voltageUnits::volt);
    }
    else if (ErrorBall1 == 0 && ErrorBall2 > 0)
    {
      // 前同色，后异色
      ToolMotor2.spin(forward, 16, voltageUnits::volt);
    }
    else if (ErrorBall1 == 0 && ErrorBall2 == 0)
    {
      // 前同色，后同色
      ToolMotor2.spin(forward, -16, voltageUnits::volt);
    }
    else if (ErrorBall1 == 0 && ErrorBall2 < 0)
    {
      // 前同色，后没球
      ToolMotor2.spin(forward, -16, voltageUnits::volt);
    }
    else if (ErrorBall1 < 0 && ErrorBall2 > 0)
    {
      // 前没球，后异色
      ToolMotor2.spin(forward, 16, voltageUnits::volt);
    }
    else if (ErrorBall1 < 0 && ErrorBall2 == 0)
    {
      // 前没球，后同色
      ToolMotor2.spin(forward, -16, voltageUnits::volt);
    }
    else if (ErrorBall1 < 0 && ErrorBall2 < 0)
    {
      // 前没球，后没球
      ToolMotor2.spin(forward, -16, voltageUnits::volt);
    }
    else
    {
      // 不可能进来，但还是留有保底
      ToolMotor2.spin(forward, 16, voltageUnits::volt);
    }
    wait(10, msec);
  }
  return 0;
}

// 模拟遥控器控制电机
void SimulatedRemoteControl(std::string input)
{
  // 吸球
  if (input == "R1" || input == "r1")
  {
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, -16, voltageUnits::volt);
    ToolMotor3.spin(forward, -10, voltageUnits::volt);
  }
  // 底桥
  else if (input == "R2" || input == "r2")
  {
    ToolMotor1.spin(forward, 16, voltageUnits::volt);
    ToolMotor2.spin(forward, 40, pct);
    ToolMotor3.spin(forward, 2, voltageUnits::volt);
  }
  // 中桥
  else if (input == "L2" || input == "l2")
  {
    ToolMotor1.spin(forward, -50, pct);
    ToolMotor2.spin(forward, 1, voltageUnits::volt);
    ToolMotor3.spin(forward, 16, voltageUnits::volt);
  }
  // 高桥
  else if (input == "L1" || input == "l1")
  {
    ToolMotor1.spin(forward, 16, voltageUnits::volt);
    ToolMotor2.spin(forward, 1, voltageUnits::volt);
    ToolMotor3.spin(forward, 16, voltageUnits::volt);
  }
  // 停止
  else if (input == "STOP" || input == "Stop" || input == "stop")
  {
    ToolMotor1.spin(forward, 0, voltageUnits::volt);
    ToolMotor2.spin(forward, 0, voltageUnits::volt);
  }
  // 递球
  else
  {
    ToolMotor1.spin(forward, 2, voltageUnits::volt);
    ToolMotor2.spin(forward, 5, pct);
    ToolMotor3.spin(forward, 30, pct);
  }
}

void AutoIntake()
{
  vex::task t(fun);
}
void StopAutoIntake()
{
  AutoIntakeFlag = false;
}
