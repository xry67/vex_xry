#ifndef CLAWCONTROLLER_H
#define CLAWCONTROLLER_H

#include "vex.h"
using namespace vex;

double moveToConNotMove(vex::motor motor_, double speed, int max_count);

class ClawController
{

public:
  bool claw_reversal = false;        // 爪子反转状态(true正，false反)
  bool claw_reversal_target = false; // 爪子的期望位置
  int claw_mod = 0;                  // 爪子模式(0关闭，1半开，2打开)
  int claw_mod_target = 1;           // 爪子开合期望位置
  double claw_half_circle = 304;     // 翻转一圈需要的度数
  double claw_open_position = 0;     // 打开时最大角度
  double claw_close_position = 0;    // 关闭的最大角度

  int claw_normal_position = 0; // 爪子的默认度数
  vex::motor reversal_motor;    // 翻转电机
  vex::motor mod_motor;         // 开合电机

  bool reversal_move_flag = false; // 爪子翻转运动标志
  bool mod_move_flag = false;      // 爪子开合运动标志

  // 构造函数
  ClawController(vex::motor reversal_motor_, vex::motor mod_motor_)
      : reversal_motor(reversal_motor_), mod_motor(mod_motor_)
  {
    reversal_motor_.setStopping(brake);
    mod_motor_.setStopping(hold);
  }

  bool init()
  {
    // 翻转限位设置===============

    claw_half_circle = moveToConNotMove(reversal_motor, 5, 20);
    wait(50, msec);
    claw_normal_position = moveToConNotMove(reversal_motor, -5, 20);

    // 开合限位设置==================================
    claw_open_position = moveToConNotMove(mod_motor, 5, 20); // 打开时最大角度
    wait(50, msec);
    claw_close_position = moveToConNotMove(mod_motor, -5, 20); // 关闭的最大角度
    std::cout << claw_open_position << " " << claw_close_position << std::endl;

    return true;
  }

  // 状态维护，多线程保护
  void maintenance()
  {
    // 翻转状态维护
    double reversal_p = 0.12;
    if (claw_reversal_target != claw_reversal)
    {
      reversalMove();
    }
    std::cout << claw_mod_target << " " << claw_mod << std::endl;
    if (claw_mod_target != claw_mod)
    {
      modMove();
    }

    // 反转维护
    if (!claw_reversal && abs(claw_normal_position - reversal_motor.position(vex::rotationUnits::deg)) >= 5)
    {
      reversal_motor.spin(forward, (claw_normal_position - reversal_motor.position(vex::rotationUnits::deg)) * reversal_p, volt);
    }
    else if (claw_reversal && abs(claw_half_circle - reversal_motor.position(vex::rotationUnits::deg)) >= 5)
    {
      reversal_motor.spin(forward, (-reversal_motor.position(vex::rotationUnits::deg) + claw_half_circle) * reversal_p, volt);
    }
    else
    {
      reversal_motor.spin(forward, 0, volt);
    }

    // 夹子状态维护
    if (claw_mod == 0)
    {
      mod_motor.setStopping(hold);
    }
    else if (claw_mod == 1)
    {
    }
    else if (claw_mod == 2)
    {
      mod_motor.setStopping(hold);
    }
  }

  // 开合控制
  bool changeMod(int target_mod)
  {
    claw_mod_target = target_mod;
    return 1;
  }

  bool addMod()
  {
    claw_mod_target = (claw_mod_target + 1) % 3;
    return 1;
  }

  bool reduceMod()
  {
    claw_mod_target -= (claw_mod_target - 1) % 3;
    return 1;
  }

  // 开合控制
  bool modMove()
  {

    double kp = 0.1;
    double ki = 0.05;
    double kd = 0.008;

    double integral = 0;
    double prevError = 0;
    double val = 0;
    double error = 5;

    int stall_count = 0;

    while (true)
    {

      // 目标位置更改
      if (claw_mod_target == 0)
      {
        val = claw_close_position;
      }
      else if (claw_mod_target == 1)
      {
        val = (claw_close_position + claw_open_position) / 2;
      }
      else
      {
        val = claw_open_position;
      }
      std::cout << mod_motor.position(deg) << " " << claw_mod_target << std::endl;

      double currentPos = mod_motor.position(rotationUnits::deg);
      double errorDist = val - currentPos;

      // 到达目标位置
      if (abs(errorDist) <= error)
      {
        break;
      }

      // PID计算（dt = 0.01秒）
      integral += errorDist * 0.01;
      double derivative = (errorDist - prevError) / 0.01;
      prevError = errorDist;

      double output = kp * errorDist + ki * integral + kd * derivative;
      // output = kp * errorDist;

      mod_motor.spin(forward, output, volt);

      // 堵转检测
      if (abs(mod_motor.velocity(dps)) <= 2)
      {
        stall_count += 1;
      }

      // 堵转退出
      if (stall_count >= 10)
      {
        break;
      }

      wait(10, msec);
    }

    mod_motor.spin(forward, 0, volt);

    // 状态维护
    claw_mod = claw_mod_target;
    std::cout << "end-->" << mod_motor.position(deg) << std::endl;
    return true;
  }

  bool reversal()
  {
    claw_reversal_target = claw_reversal_target ^ 1;
    return true;
  }

  // 翻转移动
  bool reversalMove()
  {
    reversal_move_flag = true;

    // PID参数
    double kp = 0.1;
    double ki = 0.05;
    double kd = 0.008;

    double integral = 0;
    double prevError = 0;
    double val = 0;
    double error = 5;

    int stall_count = 0;

    while (true)
    {

      // 目标位置更改
      if (!claw_reversal_target)
      {
        val = claw_normal_position;
        // std::cout << "val-->" << val << std::endl;
      }
      else
      {
        val = claw_half_circle;
        // std::cout << "val-->" << val << std::endl;
      }

      double currentPos = reversal_motor.position(rotationUnits::deg);
      double errorDist = val - currentPos;

      // 到达目标位置
      if (abs(errorDist) <= error)
      {
        break;
      }

      // PID计算（dt = 0.01秒）
      integral += errorDist * 0.01;
      double derivative = (errorDist - prevError) / 0.01;
      prevError = errorDist;

      double output = kp * errorDist + ki * integral + kd * derivative;
      // output = kp * errorDist;

      reversal_motor.spin(forward, output, volt);

      // 堵转检测
      if (abs(reversal_motor.velocity(dps)) <= 2)
      {
        stall_count += 1;
      }

      // 堵转退出
      if (stall_count >= 10)
      {
        break;
      }

      wait(10, msec);
    }

    reversal_motor.spin(forward, 0, volt);

    // 状态维护
    claw_reversal = claw_reversal_target;
    reversal_move_flag = false;
    std::cout << "end-->" << reversal_motor.position(deg) << std::endl;
    return true;
  }
};

#endif