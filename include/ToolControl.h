#ifndef TOOLCONTROL_H  
#define TOOLCONTROL_H

#include "robot-config.h"

// 工作电机相关参数
extern double ToolControl_speed1;
extern double ToolControl_speed2;
extern double ToolControl_speed3;
extern double ToolControl_timeout;
extern vex::task ToolMotorSpinTask;

int ToolMotorSpinTaskFun();

void ToolMotorSpin(double speed1, double speed2, double speed3, double timeout);

// 工作电机旋转设置
void ToolMotorSpin(double speed1, double speed2, double speed3, double timeout, bool task);

// 设置数据
void SetToolMotorSpin(double speed1, double speed2, double speed3, double timeout);

// 自动吸球
int AutoIntake();

// 分球的放置
int AutoPush();

#endif



