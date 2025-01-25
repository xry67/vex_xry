#ifndef ChassisControl_H  
#define ChassisControl_H 

#include <iostream>  
#include <chrono>  
#include <thread>  
#include "vex.h"


// 底盘运动控制算法
class ChassisControl{
  public: 

  // 底盘停止（抱死）
  void ChassisStoppingHold();
  // 底盘前进
  void ChassisForward(double v, double yaw);

  // 获取当前的位置
  double GetPositions_mean();

  // 获取当前的速度
  double GetVelocity_mean();

  // 设置电机归位 
  void ChassisRsetPosition();

  // 直线运动函数
  void Move(double v, double val, double yaw, char direction);

  // 直线运动函数(可选多线程)
  void Move(double v, double val, double yaw, char direction, bool task);

  // 旋转函数
  void Turn(double v, double val, char direction);

  // 旋转函数(可选多线程)
  void Turn(double v, double val, char direction, bool task);

  private: 
    double  Move_time_out   = 3000;  // 超时保护，默认3000毫秒
    double  Stuck_time_out  = 1000;  // 电机卡死保护，默认1000毫秒
    double  Stuck_threshold = 5;     // 电机卡死认定阈值，单位dps
    bool    Moveing         = false; // 记录是否在运动

};

//  多线程运动友元函数
int MoveTask(void* args);

// 多线程准备的结构体函数
struct ChassisMoveParams{
    ChassisControl* self;
    double v;        // 速度
    double val;      // 加速度
    double yaw;      // 减速度
    char direction;  // 运动方向
};

#endif