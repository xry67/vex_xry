#ifndef CLAWCONTROLLER_H
#define CLAWCONTROLLER_H

#include "vex.h"
using namespace vex;

class ClawController;

double moveToConNotMove(vex::motor motor_, double speed, int max_count);
double moveToConNotMove(vex::motor_group motor_, double speed, int max_count);
void task_ClawController_modMove(ClawController *claw_);

class ClawController
{

public:
  bool claw_reversal = false;        // 爪子反转状态(true正，false反)
  bool claw_reversal_target = false; // 爪子的期望位置
  int claw_mod = 1;                  // 爪子折叠模式(1平，0折叠)
  int claw_mod_target = 1;           // 爪子折叠期望位置
  double claw_half_circle = 304;     // 翻转一圈需要的度数
  double claw_open_position = 0;     // 打开时最大角度
  double claw_close_position = 0;    // 关闭的最大角度
  double claw_mid_position = 0.8;    //

  int claw_normal_position = 0; // 爪子的默认度数
  digital_out reversal_digital; // 翻转电机
  vex::motor mod_motor;         // 折叠电机
  vex::motor intake_motor;      // intake电机
  vex::motor lift_motor1;       // 抬升电机
  vex::motor lift_motor2;       // 抬升电机

  rotation Rotation; // 抬升旋转

  bool reversal_move_flag = false; // 爪子翻转运动标志
  bool mod_move_flag = false;      // 爪子开合运动标志
  bool moveing_flag = false;       // 爪子运动标志
  bool lift_move_flag = false;     // 抬升运动标志
  double StartLiftPosititon;       // 抬升初始位置

  // 构造函数
  ClawController(digital_out reversal_digital_, vex::motor mod_motor_, vex::motor intake_motor_, vex::motor lift_motor1_, vex::motor lift_motor2_, rotation Rotation_)
      : reversal_digital(reversal_digital_), mod_motor(mod_motor_), intake_motor(intake_motor_), lift_motor1(lift_motor1_), lift_motor2(lift_motor2_), Rotation(Rotation_)
  {
    mod_motor_.setStopping(coast);
    // lift_motor1.setStopping(hold);
    // lift_motor2.setStopping(hold);
  }

  bool init()
  {
    // 翻转限位设置===============
    // 开合限位设置==================================
    lift_motor1.spin(forward, 10, volt);
    lift_motor2.spin(forward, 10, volt);
    wait(500, msec);
    lift_motor1.stop();
    lift_motor2.stop();

    claw_close_position = moveToConNotMove(mod_motor, -10, 20); // 关闭的最大角度
    wait(50, msec);
    moveToConNotMove(mod_motor, 10, 20);
    motor_group lift_motor_group(lift_motor1, lift_motor2);
    moveToConNotMove(lift_motor_group, -12.8, 20);

    lift_motor1.resetPosition();
    lift_motor2.resetPosition();
    StartLiftPosititon = 0;
    claw_open_position = moveToConNotMove(mod_motor, 10, 20); // 打开时最大角度

    return true;
  }

  // 折叠前置条件满足
  void precondition()
  {
    if (lift_motor1.position(rotationUnits::deg) >= 300)
    {
      return;
    }
    LiftMove(300);
  }

  bool LiftMove(double target)
  {
    lift_move_flag = true;
    int negative = 1;
    if (lift_motor1.position(rotationUnits::deg) < target)
    {
      negative = 1;
    }
    else
    {
      negative = -1;
    }

    int stall_count = 0;

    while (true)
    {

      double currentPos = lift_motor1.position(rotationUnits::deg);

      // 到达目标位置
      if (negative == 1)
      {
        if (currentPos >= target)
        {
          break;
        }
      }
      else
      {
        if (currentPos <= target)
        {
          break;
        }
      }

      lift_motor1.spin(forward, 12.8 * negative, volt);
      lift_motor2.spin(forward, 12.8 * negative, volt);

      // 堵转检测
      if (std::abs(lift_motor1.velocity(dps)) <= 2)
      {
        stall_count += 1;
      }
      else
      {
        stall_count = 0;
      }

      // 堵转退出
      if (stall_count >= 20)
      {
        break;
      }

      wait(10, msec);
    }

    lift_motor1.spin(forward, 0, volt);
    lift_motor2.spin(forward, 0, volt);
    lift_motor1.stop();
    lift_motor2.stop();

    lift_move_flag = false;
    // std::cout << "end-->" << reversal_motor.position(deg) << std::endl;
    return true;
  }

  // 状态维护，多线程保护
  void maintenance()
  {
    if (claw_mod_target != claw_mod && !mod_move_flag)
    {
      precondition();
      task_ClawController_modMove(this);
    }

    // 夹子折叠维护
    if (!mod_move_flag)
    {

      if (claw_mod == 0)
      {
        mod_motor.spin(forward, (claw_close_position - mod_motor.position(vex::rotationUnits::deg)) * 0.0005 - 12.8 * 0.03, volt);
      }
      else if (claw_mod == 1)
      {
        mod_motor.spin(forward, (claw_open_position - mod_motor.position(vex::rotationUnits::deg)) * 0.0005 + 12.8 * 0.03, volt);
        // mod_motor.spin(forward, 12.8 * 0.03, volt);
      }

      // if (claw_mod == 0 && (abs(mod_motor.position(vex::rotationUnits::deg) - claw_close_position) >= 10))
      // {
      //   // mod_motor.spin(forward, (claw_close_position - mod_motor.position(vex::rotationUnits::deg)) * 0.2, volt);
      //   mod_motor.spin(forward, -12.8 * 0.03, volt);
      // }
      // else if (claw_mod == 1 && (abs(mod_motor.position(vex::rotationUnits::deg) - claw_open_position) >= 10))
      // {
      //   // mod_motor.spin(forward, (claw_open_position - mod_motor.position(vex::rotationUnits::deg)) * 0.2, volt);
      //   mod_motor.spin(forward, 12.8 * 0.03, volt);
      // }
      // else
      // {
      //   // mod_motor.setStopping(coast);
      //   // mod_motor.stop();
      //   mod_motor.spin(forward, 0, volt);
      // }
    }
  }

  void waitMoveStart()
  {
    for (int i = 0; i < 100; i++)
    {
      wait(10, msec);
    }
  }

  // 折叠控制
  bool changeMod()
  {

    claw_mod_target ^= 1;

    waitMoveStart();
    return 1;
  }

  bool changeMod(int target)
  {
    claw_mod_target = target;
    waitMoveStart();
    return 1;
  }

  // 折叠控制
  bool modMove()
  {
    mod_move_flag = true;
    moveing_flag = true;
    // intake_motor.spin(forward, -3, volt);
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
        val = claw_open_position;
      }
      // std::cout << mod_motor.position(deg) << " " << claw_mod_target << std::endl;

      double currentPos = mod_motor.position(rotationUnits::deg);
      double errorDist = val - currentPos;

      // 到达目标位置
      if (std::abs(errorDist) <= error)
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
      if (std::abs(mod_motor.velocity(dps)) <= 2)
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

    mod_motor.stop();

    // 状态维护
    claw_mod = claw_mod_target;
    // std::cout << "end-->" << mod_motor.position(deg) << std::endl;
    // intake_motor.spin(forward, 0, volt);
    moveing_flag = false;
    mod_move_flag = false;
    return true;
  }

  bool reversal()
  {
    reversal_digital.set(!reversal_digital.value());
    return true;
  }
};

#endif