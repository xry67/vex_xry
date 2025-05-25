#ifndef ROBOT_CONFIG_H  
#define ROBOT_CONFIG_H 

#include <vector>  
#include <functional>
#include "vex.h"
#include "AutoFun.h"
#include "PID.h"

using namespace vex;

extern int ControllerScreenWidth;  // 屏幕宽度
extern int RemoteControlMod;       // 遥控模式
extern bool RemoteControlModLock;  // ture: 不再选择遥控模式，以RemoteControlMod初始值为准，false:需要选择
extern int PowerLimit;             // 电量低警示
extern double blindCA;             // 摇杆死区
extern char stance;                // 站位选择
extern int aiVision_w;             // ai相机的宽度 
extern int aiVision_h;             // ai相机的宽度

extern double Sensitivity_Turn;           // 转向灵敏度
extern double Sensitivity_Straight;       // 直行灵敏度
extern double Sensitivity_Precision;      // 精密遥控比例


//  硬件定义
extern brain Brain;
extern controller Controller;

extern motor LeftMotor1;//14
extern motor LeftMotor2;//15
extern motor LeftMotor3;//16
extern motor_group LeftMotorGroup;  // 左电机组

extern motor RightMotor1;//18
extern motor RightMotor2;//20
extern motor RightMotor3;//19
extern motor_group RightMotorGroup;  // 右电机组

extern inertial ChassisInertial;  // 底盘陀螺仪
extern smartdrive Smartdrive;  // 官方底盘驱动

extern rotation Rotation_x;  // 底盘定位轮x
extern rotation Rotation_y;  // 底盘定位轮y

// 距离传感器
extern distance Distance_fl;  // 左前
extern distance Distance_fr;  // 右前
extern distance Distance_bl;  // 左后
extern distance Distance_br;  // 右后
extern distance Distance_l;  // 左
extern distance Distance_r;  // 右

// 视觉传感器
extern aivision::colordesc Red;
extern aivision aiVision;

extern std::vector<AutoFun> AutoFunctions;  // 自动路径储存

#endif