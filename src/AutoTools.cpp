#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"
#include <algorithm>

// 模拟遥控器控制电机
void SimulatedRemoteControl(std::string input)
{
  // // 吸球
  // if (input == "R1" || input == "r1")
  // {
  //   ToolMotor1.spin(forward, -16, voltageUnits::volt);
  //   ToolMotor2.spin(forward, -16, voltageUnits::volt);
  //   ToolMotor3.spin(forward, 2, voltageUnits::volt);
  //   digitalG.set(0);
  //   digitalH.set(0);
  //   digitalD.set(0);
  // }
  // // 底桥
  // else if (input == "R2.5" || input == "r2.5")
  // {
  //   ToolMotor1.spin(forward, 6.5, voltageUnits::volt);
  //   ToolMotor2.spin(forward, 16, voltageUnits::volt);
  //   ToolMotor3.spin(forward, 16, voltageUnits::volt);
  //   digitalH.set(0);
  //   // ToolMotor2.spin(forward, 0, voltageUnits::volt);
  // }
  // // 底桥
  // else if (input == "R2" || input == "r2")
  // {
  //   ToolMotor1.spin(forward, 5.5, voltageUnits::volt);
  //   ToolMotor2.spin(forward, 16, voltageUnits::volt);
  //   ToolMotor3.spin(forward, 16, voltageUnits::volt);
  //   digitalG.set(0);
  //   digitalH.set(0);
  //   digitalD.set(1);
  //   // ToolMotor2.spin(forward, 0, voltageUnits::volt);
  // }
  // // 中桥
  // else if (input == "L2" || input == "l2")
  // {
  //   ToolMotor1.spin(forward, -16, voltageUnits::volt);
  //   ToolMotor2.spin(forward, -16, voltageUnits::volt);
  //   ToolMotor3.spin(forward, 16, voltageUnits::volt);
  //   digitalG.set(1);
  //   digitalD.set(0);
  //   digitalH.set(0);
  // }
  // // 中桥
  // else if (input == "L2-" || input == "l2-")
  // {
  //   ToolMotor1.spin(forward, -16, voltageUnits::volt);
  //   ToolMotor2.spin(forward, -16, voltageUnits::volt);
  //   ToolMotor3.spin(forward, 16, voltageUnits::volt);
  //   digitalG.set(1);
  //   digitalD.set(0);
  //   digitalH.set(0);
  // }
  // // 高桥
  // else if (input == "L1" || input == "l1")
  // {
  //   ToolMotor1.spin(forward, -16, voltageUnits::volt);
  //   ToolMotor2.spin(forward, -16, voltageUnits::volt);
  //   ToolMotor3.spin(forward, -16, voltageUnits::volt);
  //   digitalG.set(0);
  //   digitalD.set(0);
  //   digitalH.set(0);
  // }
  // // 停止
  // else if (input == "STOP" || input == "Stop" || input == "stop")
  // {
  //   ToolMotor1.spin(forward, 0, voltageUnits::volt);
  //   ToolMotor2.spin(forward, 0, voltageUnits::volt);
  //   ToolMotor3.spin(forward, 0, voltageUnits::volt);
  //   digitalH.set(0);
  // }
  // // 递球
  // else
  // {
  //   ToolMotor1.spin(forward, 2, voltageUnits::volt);
  //   ToolMotor2.spin(forward, 5, pct);
  // }
}

void ChassisForward(double angle, double speed, double time)
{
  ChassisControlSTS.ChassisForwardAngle(angle);
  for (int i = 0; i < time / 10; i++)
  {
    ChassisControlSTS.ChassisForward(speed);
    wait(10, msec);
  }
}