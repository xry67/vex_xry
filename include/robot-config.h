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
extern double StraightCorrectionSpeed;   // 执行修正pid最大速度

extern double Sensitivity_Turn;           // 转向灵敏度
extern double Sensitivity_Straight;       // 直行灵敏度
extern double Sensitivity_Precision;      // 精密遥控比例

extern double TrackSpeed;  // 履带速度

// 遥控期间自动追踪的灵敏度
extern double Auto_Turn_To_Target_Speed;


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

extern motor TrackLeft;    // 左履带
extern motor TrackRight;  // 右履带
extern motor Flywheel;  // 尾部飞轮
extern digital_out Uplift; // 抬升

extern inertial ChassisInertial;  // 底盘陀螺仪
extern smartdrive Smartdrive;  // 官方底盘驱动
 
extern rotation Rotation_x;  // 底盘定位轮x
extern rotation Rotation_y;  // 底盘定位轮y

extern optical Optical;      // 颜色传感器

// 距离传感器
extern distance Distance_fl;  // 左前
extern distance Distance_fr;  // 右前
extern distance Distance_bl;  // 左后
extern distance Distance_br;  // 右后
extern distance Distance_l;  // 左
extern distance Distance_r;  // 右

// 视觉传感器
extern aivision::colordesc Red;
extern aivision::colordesc Bule;
extern aivision::colordesc Orange;
extern aivision aiVision;

extern std::vector<AutoFun> AutoFunctions;  // 自动路径储存

extern std::vector<char*> MotorNames;
extern std::vector<motor> Motors;

extern std::vector<char*> DistanceNames;
extern std::vector<distance> Distances;

#endif