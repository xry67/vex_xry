#ifndef POSITIONING_H
#define POSITIONING_H 

#include <iostream>  
#include <chrono>  
#include <thread>  
#include "vex.h"


class Positioning
{
  public: 
  double x = 0.0; // x坐标
  double y = 0.0; // y坐标
  double lest_d = 0.0;

  void Reset();  // 重置位置信息和陀螺仪角度
  void TurnOn();   // 定位开始

  // 获得与x轴正方向的夹角
  double GetAngle(double x, double y);
  double GetAngle();

};

extern Positioning PositioningSTS;


#endif