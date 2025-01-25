#include <iostream>  
#include <chrono>  
#include <algorithm> // 包含 min 和 max 函数
#include <thread>  
#include "vex.h"
#include "PID.h"
#include "ChassisControl.h"

// 底盘运动控制算法

  // 底盘停止（抱死）
  void ChassisControl::ChassisStoppingHold(){
      ChassisForward(0, 0);
      LeftMotor1.setStopping(hold);
      LeftMotor2.setStopping(hold);
      LeftMotor3.setStopping(hold);
      RightMotor1.setStopping(hold);
      RightMotor2.setStopping(hold);
      RightMotor3.setStopping(hold);
  }

  // 底盘前进
  void ChassisControl::ChassisForward(double v, double yaw = 0){
      
      double left_yaw  = 1.0;
      double right_yaw = 1.0;

      // 左转
      if (yaw < 0){
          left_yaw = 2.0 * yaw + 1;
      }
      // 右转
      if (yaw > 0){
          right_yaw = -2.0 * yaw + 1;
      }


      LeftMotor1.spin(vex::forward, v * left_yaw, dps);
      LeftMotor2.spin(vex::forward, v * left_yaw, dps);
      LeftMotor3.spin(vex::forward, v * left_yaw, dps);
      RightMotor1.spin(vex::forward, v * right_yaw, dps);
      RightMotor2.spin(vex::forward, v * right_yaw, dps);
      RightMotor3.spin(vex::forward, v * right_yaw, dps);
  }

  // 获取当前的位置
  double ChassisControl::GetPositions_mean(){
      double ans = 0.0;

      ans += std::abs(LeftMotor1.position(vex::rotationUnits::deg));
      ans += std::abs(LeftMotor2.position(vex::rotationUnits::deg));
      ans += std::abs(LeftMotor3.position(vex::rotationUnits::deg));
      ans += std::abs(RightMotor1.position(vex::rotationUnits::deg));
      ans += std::abs(RightMotor2.position(vex::rotationUnits::deg));
      ans += std::abs(RightMotor3.position(vex::rotationUnits::deg));
      return ans / 6;
  }

  // 获取当前的速度
  double ChassisControl::GetVelocity_mean(){
      double ans = 0.0;

      ans += std::abs(LeftMotor1.velocity(dps));
      ans += std::abs(LeftMotor2.velocity(dps));
      ans += std::abs(LeftMotor3.velocity(dps));
      ans += std::abs(RightMotor1.velocity(dps));
      ans += std::abs(RightMotor2.velocity(dps));
      ans += std::abs(RightMotor3.velocity(dps));
      return ans / 6;
  }

  // 设置电机归位 
  void ChassisControl::ChassisRsetPosition(){
      LeftMotor1.resetPosition();
      RightMotor1.resetPosition();
      LeftMotor2.resetPosition();
      RightMotor2.resetPosition();
      LeftMotor3.resetPosition();
      RightMotor3.resetPosition();
  }

  // 直线运动函数
  // bdqpnu
  void ChassisControl::Move(double v, double val, double yaw, char direction){
      // 最大速度
      // 运动距离
      // 偏转
      // 方向
      // 多线程
      ChassisRsetPosition();  // 电机初始化
 
      double dt = 0.01;
      double stuck_time_count = 0;  // 电机卡死时间统计
      double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
      double move_time_out = Move_time_out + (val / v * 1000);
      int    negative = 1;   // 负方向指示器
      int yaw_negative= 1;   // 负偏指示器


      if (direction == 'u'){
          negative = -1;
          yaw_negative = 1;
          yaw = 0;
      }
      else if (direction == 'n'){
          negative = 1;
          yaw_negative = 1;
          yaw = 0;
      }
      else if (direction == 'p'){
          negative = 1;
          yaw_negative = 1;
      }
      else if (direction == 'q'){
          negative = 1;
          yaw_negative = -1;
      }
      else if (direction == 'b'){
          negative = -1;
          yaw_negative = 1;
      }
      else if (direction == 'd'){
          negative = -1;
          yaw_negative = -1;
      }


      PID pid(5, 0.001, 0.1); // 创建PID控制器实例 
      pid.setSetpoint(val); // 设置目标位置

      Moveing = true;  // 当前在运动

      while (true) {  
          double current_position = GetPositions_mean();  
          std::cout << "P: " << LeftMotor1.position(degrees);
          std::cout << "\t\t V: " << LeftMotor1.velocity(dps) << std::endl;
          double output = pid.calculate(current_position, dt);  
          // 控制小车运动 
          ChassisForward(std::min(v, output) * negative, yaw * yaw_negative); 

          // 检查目标是否达到  
          if (fabs(current_position - val) <= 0.1) {   
              break;  
          }  

          // 算法收敛保护
          if (output <= Stuck_threshold){
            break;
          }

          // 电机抱死检测
          if (GetVelocity_mean() <= Stuck_threshold){
              stuck_time_count += dt * 1000.0;  // 计算抱死时间
              if (stuck_time_count >= Stuck_time_out){
                std::cout << "stuck_time_count" << std::endl;
                break;
              }
          }
          else{
            stuck_time_count = 0; // 不抱死则刷新计数
          }

          // 超时保护
          double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();// 当前时间
          if (now_time - start_time >= move_time_out / 1000){
              std::cout << "timeout" << std::endl;
              break;
          }

          wait(dt * 1000.0, msec);
      }  

      ChassisStoppingHold();
      Moveing = false;  // 运动停止了
  
  
  }

  // 直线运动函数(可选多线程)
  void ChassisControl::Move(double v, double val, double yaw, char direction, bool task){

    if (task == true){
        ChassisMoveParams* Params = new ChassisMoveParams;
        Params->self = this;
        Params->v = v;
        Params->val = val;
        Params->yaw = yaw;
        Params->direction = direction;

        void* p = Params;
        vex::task Task(MoveTask, p);
    }
    else{
        Move(v, val, yaw, direction, yaw);    
    }
  }

  //  多线程运动友元函数
int MoveTask(void* args){
    ChassisMoveParams* params = (ChassisMoveParams*)args;
    ChassisControl* self= params->self;       // 当前类
    double v            = params->v;          // 速度
    double val          = params->val;        // 路径长度
    char direction      = params->direction;  // 运动方向
    double yaw          = params->yaw;        // 偏转
    self->Move(v, val, direction, yaw, yaw);
    delete args;
    return 0;
}