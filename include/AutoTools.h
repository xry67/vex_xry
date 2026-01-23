#ifndef AUTOTOOLS_H
#define AUTOTOOLS_H

#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void intake_and_pushback(double angle, int intake_time, int push_time);

void move_intake_time(double angle, double intake_time);

void Move_u_Long(double angle);

// 与intake_and_pushback相比没有转向
void intake_and_pushback_new(double angle, int intake_time, int push_time);

// 等待前方距离传感器到达某个时间，然后停止运动
void WaitFrontDistanceArriveThenStopMove(double speed, double val, double angle, double Distance);

void AutoIntake();

// 钩子
void Hook(double angle);

// 模拟遥控器控制电机
void SimulatedRemoteControl(std::string input);

void StopAutoIntake();
#endif