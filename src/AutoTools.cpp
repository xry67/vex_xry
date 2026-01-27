#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"
#include <algorithm>

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

void GoToMiddleBridge_Below(double angle)
{

  ChassisControlSTS.Over_count = 10;
  ChassisControlSTS.TurnTo(10, angle, 0);
  ChassisControlSTS.Over_count = 5;
  // 735
  // 1208

  double Left_d = GetDistance_left() + l_w;
  double Back_d = GetDistance_back() + b_h;

  ChassisControlSTS.MoveToDistance(5, Left_d, angle, 'f');

  ChassisControlSTS.TurnTo(10, angle + 45, 0);

  digitalB.set(0);
  digitalC.set(0);

  ChassisControlSTS.Move(10, GetMoveLen(Left_d, Back_d) * 735 / 1208 * 1.5, angle + 45, 'n', 1);
  ChassisControlSTS.WaitPositionArrive(GetMoveLen(Left_d, Back_d) * 735 / 1208 * 0.7);
  ChassisControlSTS.ResetMaxSpeed(3);
  ChassisControlSTS.ReSetToEncoderWheel(1500, angle + 45, 'n');
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
  for (int i = 0; i < 200; i++)
  {
    if (GetDistance_front() <= 250)
    {
      break;
    }
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

  // 提前打球
  ChassisControlSTS.WaitPositionArrive(1100);

  ChassisControlSTS.SetStopModCoast();
  ChassisControlSTS.StopMoving();
  // 努力进入对位
  ChassisControlSTS.ChassisLeftControl(-0.5);
  ChassisControlSTS.ChassisRightControl(-0.5);
  wait(push_time, msec);
  ChassisControlSTS.SetStopModHold();
  ToolMotor2.spin(forward, 0, voltageUnits::volt);
  ToolMotor3.spin(forward, 0, voltageUnits::volt);
}

void move_intake_time(double angle, double intake_time)
{
  // AutoIntake();
  ChassisControlSTS.ChassisForwardAngle(angle);

  bool now_mod = 1;
  int k = 10;

  for (int i = 0; i < 200; i++)
  {
    ChassisControlSTS.ChassisForward(4);
    if (GetDistance_front() <= 280)
    {
      break;
    }

    if (GetDistance_front() > 280 && std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 5)
    {
      k = 25;
      intake_time += 1000;
      break;
    }
    wait(10, msec);
  }

  // 轻微的蠕动
  for (int i = 0; i < intake_time / 10; i++)
  {
    // GetDistance_front() > 217

    if (now_mod)
    {
      ChassisControlSTS.ChassisForward(4);
    }
    else
    {
      ChassisControlSTS.ChassisForward(-1);
    }

    // if (i % k == 0)
    // {
    //   now_mod ^= 1;
    // }

    wait(10, msec);
  }
}

void Move_u_Long_r(double angle)
{
  // ChassisControlSTS.Move(10, 1000, angle + 2, 'n'); // 1550
  // Brain.Screen.printAt(150, 150, "%f", GetDistance_right());
  ChassisControlSTS.Move(5, 1000, angle - (GetDistance_right() - 430) * 0.15, 'u', true); // 1550
  // ChassisControlSTS.Move(5, 700, angle, 'u', true); // 1550
  ChassisControlSTS.WaitPositionArrive(400);
  ChassisControlSTS.ResetStraightAngle(angle);
  while (ChassisControlSTS.IsMoving())
  {
    if (std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 3)
    {
      ChassisControlSTS.StopMoving();
    }
    wait(10, msec);
  }
  ChassisControlSTS.WaitMoveFinish();
}

void Move_u_Long(double angle)
{
  // ChassisControlSTS.Move(10, 1000, angle + 2, 'n'); // 1550
  ChassisControlSTS.Move(5, 700, angle + (GetDistance_left() - 430) * 0.15, 'u', true); // 1550
  ChassisControlSTS.WaitPositionArrive(400);
  ChassisControlSTS.ResetStraightAngle(angle);
  while (ChassisControlSTS.IsMoving())
  {
    if (std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 3)
    {
      ChassisControlSTS.StopMoving();
    }
    wait(10, msec);
  }
  ChassisControlSTS.WaitMoveFinish();
}

void intake_and_pushback_new_r(double angle, int intake_time, int push_time)
{
  // 左侧最优距离：500
  // 吸导入
  SimulatedRemoteControl("R1");

  // ChassisControlSTS.Move(10, 1000, angle + 2, 'n'); // 1550
  ChassisControlSTS.Move(5, 1700, angle + (GetDistance_right() - 470) * 0.12, 'n', true); // 1550
  ChassisControlSTS.WaitPositionArrive(500);
  ChassisControlSTS.ResetStraightAngle(angle);

  // 使用距离传感器判断是否到达恒力区域
  while (GetDistance_front() > 350)
  {
    wait(10, msec);
  }
  ChassisControlSTS.SetStopModCoast(); // 需要保留，否则容易插不进去
  ChassisControlSTS.StopMoving();

  move_intake_time(angle, intake_time);

  // StopAutoIntake();

  // 倒车 打球
  ChassisControlSTS.Move(7, 1500, angle, 'u', true); // 1550
  ChassisControlSTS.WaitPositionArrive(500);

  // 提前打球
  ChassisControlSTS.WaitPositionArrive(1200);
  digitalE.set(0);

  while (ChassisControlSTS.IsMoving())
  {
    if (std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 3)
    {
      ChassisControlSTS.StopMoving();
      break;
    }
    wait(10, msec);
  }

  SimulatedRemoteControl("L1");

  ChassisControlSTS.SetStopModCoast();
  ChassisControlSTS.StopMoving();
  // 努力进入对位
  ChassisControlSTS.ChassisForwardAngle(angle);
  ChassisControlSTS.ChassisForward(-2);
  wait(push_time, msec);
}

void intake_and_pushback_new(double angle, int intake_time, int push_time)
{
  // 左侧最优距离：500
  // 吸导入
  SimulatedRemoteControl("R1");

  // ChassisControlSTS.Move(10, 1000, angle + 2, 'n'); // 1550
  ChassisControlSTS.Move(5, 1700, angle + (470 - GetDistance_left()) * 0.12, 'n', true); // 1550
  ChassisControlSTS.WaitPositionArrive(500);
  ChassisControlSTS.ResetStraightAngle(angle);

  // 使用距离传感器判断是否到达恒力区域
  while (GetDistance_front() > 350)
  {
    wait(10, msec);
  }
  ChassisControlSTS.SetStopModCoast(); // 需要保留，否则容易插不进去
  ChassisControlSTS.StopMoving();

  move_intake_time(angle, intake_time);

  // StopAutoIntake();

  // 倒车 打球
  ChassisControlSTS.Move(7, 1500, angle, 'u', true); // 1550
  ChassisControlSTS.WaitPositionArrive(500);

  // 提前打球
  ChassisControlSTS.WaitPositionArrive(1200);
  digitalE.set(0);

  while (ChassisControlSTS.IsMoving())
  {
    if (std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 3)
    {
      ChassisControlSTS.StopMoving();
      break;
    }
    wait(10, msec);
  }

  SimulatedRemoteControl("L1");

  ChassisControlSTS.SetStopModCoast();
  ChassisControlSTS.StopMoving();
  // 努力进入对位
  ChassisControlSTS.ChassisForwardAngle(angle);
  ChassisControlSTS.ChassisForward(-2);
  wait(push_time, msec);
}

// 钩子
void Hook(double angle)
{
  ChassisControlSTS.Move_PPS = {0.0182, 0.5, 2};
  ChassisControlSTS.Move(16, 400, 'n'); // 1550

  ChassisControlSTS.AutoPPSTurnTo(10, angle - 30);
  ChassisControlSTS.CurveMove(10, 550, 580, 'd', true);
  ChassisControlSTS.WaitPositionArrive(400);
  ChassisControlSTS.StopMoving();

  ChassisControlSTS.Over_count = 5;
  ChassisControlSTS.Turn_PPS = {0.65, 0.2, 2};
  ChassisControlSTS.min_turn_speed = 1.9;
  ChassisControlSTS.TurnTo(2.7, angle, false);

  SimulatedRemoteControl("Stop");
  wait(200, msec);
  ChassisControlSTS.ChassisLeftControl_pct(-5);
  ChassisControlSTS.ChassisRightControl_pct(-5);
  wait(1300, msec);

  // 急刹车
  ChassisControlSTS.ChassisLeftControl(1);
  ChassisControlSTS.ChassisRightControl(1);
  wait(500, msec);
  ChassisControlSTS.ChassisStoppingHold();
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
    ToolMotor3.stop();
    ToolMotor3.setStopping(hold);
    digitalG.set(0);
    digitalH.set(0);
    digitalD.set(0);
  }
  // 底桥
  else if (input == "R2" || input == "r2")
  {
    ToolMotor1.spin(forward, 6.5, voltageUnits::volt);
    ToolMotor2.spin(forward, 16, voltageUnits::volt);
    ToolMotor3.spin(forward, 16, voltageUnits::volt);
    digitalG.set(0);
    digitalH.set(0);
    digitalD.set(1);
    // ToolMotor2.spin(forward, 0, voltageUnits::volt);
  }
  // 中桥
  else if (input == "L2" || input == "l2")
  {
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, -16, voltageUnits::volt);
    ToolMotor3.spin(forward, 16, voltageUnits::volt);
    digitalG.set(1);
    digitalD.set(0);
  }
  // 中桥
  else if (input == "L2-" || input == "l2-")
  {
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, -16, voltageUnits::volt);
    ToolMotor3.spin(forward, 16, voltageUnits::volt);
    digitalG.set(1);
    digitalD.set(0);
  }
  // 高桥
  else if (input == "L1" || input == "l1")
  {
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, -16, voltageUnits::volt);
    ToolMotor3.spin(forward, -16, voltageUnits::volt);
    digitalG.set(0);
    digitalD.set(0);
  }
  // 停止
  else if (input == "STOP" || input == "Stop" || input == "stop")
  {
    ToolMotor1.spin(forward, 0, voltageUnits::volt);
    ToolMotor2.spin(forward, 0, voltageUnits::volt);
    ToolMotor3.spin(forward, 0, voltageUnits::volt);
  }
  // 递球
  else
  {
    ToolMotor1.spin(forward, 2, voltageUnits::volt);
    ToolMotor2.spin(forward, 5, pct);
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
