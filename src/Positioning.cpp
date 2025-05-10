#include <iostream>  
#include <chrono>  
#include <thread>  
#include <cmath>  // 包含数学函数
#include "vex.h"
#include "Positioning.h"
#include "robot-config.h"

Positioning PositioningSTS = Positioning();

void ChangeXY(){
  while(PositioningSTS.flag){

    // std::cout << ChassisInertial.rotation() << std::endl;

    double angle = ChassisInertial.rotation(deg);  // 获取角度
    // 角度转换为弧度  
    double angle_radians = angle * M_PI / 180.0; 

    double cos_value = cos(angle_radians);   
    double sin_value = sin(angle_radians);
    double now_xd = Rotation_x.position(deg);
    double now_yd = Rotation_y.position(deg);

    double dx = now_xd - PositioningSTS.lest_xd;

    double dy = now_yd - PositioningSTS.lest_yd;
    PositioningSTS.lest_xd = now_xd;
    PositioningSTS.lest_yd = now_yd;

    std::cout << "x" << now_xd << " " << now_yd << " " << angle << std::endl;

    PositioningSTS.x += cos_value * dx - sin_value * dy;
    PositioningSTS.y += -1 * sin_value * dx + cos_value * dy;
    
    std::cout << PositioningSTS.x << " " << PositioningSTS.y << std::endl;

    wait(10, msec);
  }
}

void Positioning::TurnOFF(){
  this->flag = false;
}

// 重置位置信息和陀螺仪角度
void Positioning::Reset(){
  this->x = 0.0;
  this->y = 0.0;
  this->lest_xd = 0.0;
  this->lest_yd = 0.0;
  ChassisInertial.resetRotation();  // 陀螺仪初始化
  Rotation_x.resetPosition();       // 定位轮初始化
  Rotation_y.resetPosition();       // 定位轮初始化
}

// 定位开始
void Positioning::TurnOn(){
  this->flag = true;
  vex::thread Task(ChangeXY);  //多线程执行
} 


// 获得与x轴正方向的夹角
double Positioning::GetAngle(double x, double y)
{
  double angle_rad = atan2(x, y);  // 计算夹角，单位是弧度  
  double angle_deg = angle_rad * 180.0 / M_PI;  // 转成角度 
  return angle_deg;
}

double Positioning::GetAngle(){
  return GetAngle(this->x, this->y);
}


