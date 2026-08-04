#ifndef CLAWCONTROLLER_H
#define CLAWCONTROLLER_H

#include "vex.h"

using namespace vex;

class ClawController;

double moveToConNotMove(vex::motor motor_, double speed, int max_count);
void task_ClawController_reversalMove(ClawController *claw_);
void task_ClawController_modMove(ClawController *claw_);
void task_ClawController_liftMove(ClawController *claw_, double lift_target_);

class ClawController
{

public:
  bool claw_reversal = false;        // 爪子反转状态(true正，false反)
  bool claw_reversal_target = false; // 爪子的期望位置
  int claw_mod = 0;                  // 爪子折叠模式(0平，1中间，2折叠)
  int claw_mod_target = 0;           // 爪子折叠期望位置
  double claw_half_circle = 304;     // 翻转一圈需要的度数
  double claw_open_position = 0;     // 打开时最大角度
  double claw_close_position = 0;    // 关闭的最大角度
  double claw_mid_position = 0.8;    //

  int claw_normal_position = 0; // 爪子的默认度数
  vex::motor reversal_motor;    // 翻转电机
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
  ClawController(vex::motor reversal_motor_, vex::motor mod_motor_, vex::motor intake_motor_, vex::motor lift_motor1_, vex::motor lift_motor2_, rotation Rotation_, double start_lift_position)
      : reversal_motor(reversal_motor_), mod_motor(mod_motor_), intake_motor(intake_motor_), lift_motor1(lift_motor1_), lift_motor2(lift_motor2_), Rotation(Rotation_), StartLiftPosititon(start_lift_position)
  {
    reversal_motor_.setStopping(brake);
    mod_motor_.setStopping(hold);
    lift_motor1.setStopping(hold);
    lift_motor2.setStopping(hold);
  }

  bool init()
  {
    LiftMove_Normalized(0.5);
		wait(300, msec);
    // 翻转限位设置===============

    claw_half_circle = moveToConNotMove(reversal_motor, 5, 20);
    wait(50, msec);
    claw_normal_position = moveToConNotMove(reversal_motor, -5, 20);

    // 开合限位设置==================================
    claw_open_position = moveToConNotMove(mod_motor, 5, 20); // 打开时最大角度
    wait(50, msec);
    claw_close_position = moveToConNotMove(mod_motor, -5, 20); // 关闭的最大角度
    // std::cout << claw_open_position << " " << claw_close_position << std::endl;

    claw_mid_position = claw_close_position + (claw_open_position - claw_close_position) * 1;

    return true;
  }

  // 折叠前置条件满足
  void precondition()
  {
    if (Rotation.angle() >= StartLiftPosititon + 30)
    {
      return;
    }
    LiftMove(StartLiftPosititon + 30);
  }

  // target：高度（取值最下38-最上153）
  bool LiftMove(double target, double error)
  {
    lift_move_flag = true;

    // PID参数
    double kp = 0;
    double ki = 0;
    double kd = 0;

    // 两套pid选择
    if (Rotation.angle() < target)
    {
      kp = 0.5;
      ki = 0.5;
      kd = 0.05;
    }
    else
    {
      kp = 0.5;
      ki = 0.5;
      kd = 0.05;
    }

		double absolute_offset = -3.2;

    double integral = 0;
    double prevError = 0;
    double val = target;

    int stall_count = 0;
    int over_count = 0;

    while (true)
    {

      double currentPos = Rotation.angle();
      double errorDist = val - currentPos;

      // 到达目标位置
      if (std::abs(errorDist) <= error)
      {
        over_count += 1;
        if (over_count >= 5)
        {
          break;
        }
      }
      else
      {
        over_count = 0;
      }

      // PID计算（dt = 0.01秒）
      integral += errorDist * 0.01;
      double derivative = (errorDist - prevError) / 0.01;
      prevError = errorDist;

      double output = kp * errorDist + ki * integral + kd * derivative;
      // output = kp * errorDist;

      lift_motor1.spin(forward, -output + absolute_offset, volt);
      lift_motor2.spin(forward, -output + absolute_offset, volt);

      // 堵转检测
      if (std::abs(lift_motor1.velocity(dps)) <= 2)
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

    lift_motor1.spin(forward, 0, volt);
    lift_motor2.spin(forward, 0, volt);
    lift_motor1.stop();
    lift_motor2.stop();

    lift_move_flag = false;
    // std::cout << "end-->" << reversal_motor.position(deg) << std::endl;
    return true;
  }

	bool LiftMove(double target) {
		return LiftMove(target, 1.0);
	}

	// target：高度（取值最下0.00-最上1.00）
	bool LiftMove_Normalized(double target, bool task) {
		if (!task){
			return LiftMove(target * 115.0 + this->StartLiftPosititon, 5.0);
		}else{
			task_ClawController_liftMove(this, target);
		}
		return false;
	}

	bool LiftMove_Normalized(double target) {
		return LiftMove(target * 115.0 + this->StartLiftPosititon, 5.0); // WTF Spelling 神了
	}

  // 状态维护，多线程保护
  void maintenance()
  {
    // 翻转状态维护
    double reversal_p = 0.12;
    // std::cout << "claw_reversal_target" << claw_reversal_target << " " << claw_reversal << std::endl;
    if (claw_reversal_target != claw_reversal && !reversal_move_flag)
    {
      task_ClawController_reversalMove(this);
    }
    // std::cout << "claw_mod_target" << claw_mod_target << " " << claw_mod << std::endl;
    if (claw_mod_target != claw_mod && !mod_move_flag)
    {
      // precondition();
      std::cout << "claw_mod_target 1" << std::endl;
      task_ClawController_modMove(this);
    }

    // 夹子折叠维护
    if (!mod_move_flag)
    {
      if (claw_mod == 0 && ((mod_motor.position(vex::rotationUnits::deg) - claw_close_position) >= 10))
      {
        mod_motor.spin(forward, (claw_close_position - mod_motor.position(vex::rotationUnits::deg)) * 0.0005 - 12.8 * 0.03, volt);
      }
      else if (claw_mod == 2 && ((mod_motor.position(vex::rotationUnits::deg) - claw_open_position) >= 10))
      {
        mod_motor.spin(forward, (claw_open_position - mod_motor.position(vex::rotationUnits::deg)) * 0.0005 + 12.8 * 0.03, volt);
      }
      else if (claw_mod == 1 && ((mod_motor.position(vex::rotationUnits::deg) - claw_mid_position) >= 10))
      {
        mod_motor.spin(forward, (claw_mid_position - mod_motor.position(vex::rotationUnits::deg)) * 0.2, volt);
      }
      else
      {
        mod_motor.setStopping(hold);
        mod_motor.stop();
      }
    }

    // 电梯高度维护
    // if (!lift_move_flag)
    // {
    //   if (claw_mod == 1)
    //   {
    //     double currentPos = Rotation.angle();
    //     double target = StartLiftPosititon + 22;

    //     if (std::abs(currentPos - target) > 1.5)
    //     {
    //       double p = 12;
    //       lift_motor1.spin(forward, (currentPos - target) * 2, volt);
    //       lift_motor2.spin(forward, (currentPos - target) * 2, volt);
    //     }
    //     else
    //     {
    //       lift_motor1.stop();
    //       lift_motor2.stop();
    //     }
    //   }
    // }
  }

  void waitMoveStart()
  {
    for (int i=0; i<1000; i++) 
    {
      if (moveing_flag){
        break;
      }
      wait(10, msec);
    }
  }

  // 折叠控制
  bool changeMod()
  {
    if (claw_mod_target >= 1)
    {
      claw_mod_target = 0;
    }
    else
    {
      claw_mod_target = 2;
    }
    waitMoveStart();
    return 1;
  }

  bool changeMod(int target)
  {
    claw_mod_target = target;
    // waitMoveStart();
    return 1;
  }

  // 折叠控制
  bool modMove()
  {
    std::cout << "claw_mod_target 2" << std::endl;
    mod_move_flag = true;
    moveing_flag = true;
    intake_motor.spin(forward, -3, volt);
    double kp = 0.1;
    double ki = 0.05;
    double kd = 0.008;

    double integral = 0;
    double prevError = 0;
    double val = 0;
    double error = 20;

    int stall_count = 0;

    while (true)
    {
      // mod_move_flag = true;
      // moveing_flag = true;
      // 目标位置更改
      if (claw_mod_target == 0)
      {
        val = claw_close_position;
      }
      else if (claw_mod_target == 1)
      {
        val = claw_mid_position;
      }
      else if (claw_mod_target == 2)
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

      // mod_motor.spin(forward, output, volt);
      if (val == claw_mid_position)
      {
        mod_motor.spin(forward, output, volt);
      }
      else
      {
        mod_motor.spin(forward, output, volt);
        std::cout << "[[ ]]=== " << output << " " << claw_close_position << " " << claw_open_position << " " << currentPos << std::endl;
      }

      // 堵转检测
      if ((mod_motor.velocity(dps)) <= 2)
      {
        stall_count += 1;
      }
      else
      {
        stall_count = 0;
      }

      // 堵转退出
      if (stall_count >= 100)
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
    // std::cout << "reversal:" << claw_reversal_target << " " << claw_reversal << std::endl;
    claw_reversal_target = claw_reversal_target ^ 1;
    // waitMoveStart();
    return true;
  }

  // 翻转移动
  bool reversalMove()
  {
    moveing_flag = true;
    intake_motor.spin(forward, -3, volt);
    reversal_move_flag = true;

    // PID参数
    double kp = 0.2;
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

      reversal_motor.spin(forward, output, volt);
      std::cout << "[[ ]] " << output << " " << claw_normal_position << " " << claw_half_circle << " " << currentPos << std::endl;

      // 堵转检测
      if ((reversal_motor.velocity(dps)) <= 2)
      {
        stall_count += 1;
      }
      else
      {
        stall_count = 0;
      }

      // 堵转退出
      if (stall_count >= 100)
      {
        break;
      }

      wait(10, msec);
    }

    reversal_motor.spin(forward, 0, volt);
    reversal_motor.stop();

    // 状态维护
    claw_reversal = claw_reversal_target;
    reversal_move_flag = false;
    // std::cout << "end-->" << reversal_motor.position(deg) << std::endl;
    // intake_motor.spin(forward, 0, volt);
    moveing_flag = false;
    return true;
  }
};

#endif