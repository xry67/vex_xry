#include <iostream>  
#include <chrono>  
#include <thread>  
#include <cmath>  // 包含数学函数
#include "vex.h"
#include "Positioning.h"
#include "robot-config.h"

Positioning PositioningSTS = Positioning();

void ChangeXY(){
  while(true){

    // std::cout << ChassisInertial.rotation() << std::endl;

    double angle = ChassisInertial.rotation(deg);  // 获取角度
    // 角度转换为弧度  
    double angle_radians = angle * M_PI / 180.0; 

    double cos_value = cos(angle_radians);   
    double sin_value = sin(angle_radians);
    double now_d = Rotation_x.position(deg);
    double d = now_d - PositioningSTS.lest_d;
    PositioningSTS.lest_d = now_d;


    PositioningSTS.x += cos_value * d;
    PositioningSTS.y += sin_value * d;
    
    // std::cout << PositioningSTS.x << " " << PositioningSTS.y << std::endl;

    wait(10, msec);
  }
}

// 重置位置信息和陀螺仪角度
void Positioning::Reset(){
  this->x = 0.0;
  this->y = 0.0;
  this->lest_d = 0.0;
  ChassisInertial.resetRotation();  // 陀螺仪初始化
  Rotation_x.resetPosition();       // 定位轮初始化
}

// 定位开始
void Positioning::TurnOn(){
  vex::thread Task(ChangeXY);  //多线程执行
} 


// 获得与x轴正方向的夹角
double Positioning::GetAngle(double x, double y)
{
  double angle_rad = atan2(y, x);  // 计算夹角，单位是弧度  
  double angle_deg = angle_rad * 180.0 / M_PI;  // 转成角度 
  return angle_deg;
}

double Positioning::GetAngle(){
  return GetAngle(this->x, this->y);
}


