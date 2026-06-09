#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <vector>
#include <functional>
#include "vex.h"
#include "AutoFun.h"
#include "PPS.h"
#include "ChassisRemoteControl.h"
#include "ClawController.h"
#include <map>

using namespace vex;

extern int ControllerScreenWidth;      // 屏幕宽度
extern int PowerLimit;                 // 电量低警示
extern double blindCA;                 // 摇杆死区
extern char stance;                    // 站位选择
extern int aiVision_w;                 // ai相机的宽度
extern int aiVision_h;                 // ai相机的宽度
extern double StraightCorrectionSpeed; // 执行修正pps最大速度

extern double Sensitivity_Turn;      // 转向灵敏度
extern double Sensitivity_Straight;  // 直行灵敏度
extern double Sensitivity_Precision; // 精密遥控比例
extern int auto_index;               // 自动选择

extern bool claw_reversal; // 爪子反转状态(true正，false反)
extern int claw_mod;       // 爪子模式(0关闭，1半开，2打开)

// 遥控期间自动追踪的灵敏度
extern double Auto_Turn_To_Target_Speed;

//  硬件定义
extern brain Brain;
extern controller Controller;
extern competition Competition;

extern motor LeftMotor1;           // 14
extern motor LeftMotor2;           // 15
extern motor LeftMotor3;           // 16
extern motor_group LeftMotorGroup; // 左电机组

extern motor RightMotor1;           // 18
extern motor RightMotor2;           // 20
extern motor RightMotor3;           // 19
extern motor_group RightMotorGroup; // 右电机组

extern smartdrive Smartdrive;

extern motor ToolMotor1; //
extern motor ToolMotor2; //
extern motor ToolMotor3; //
extern motor ToolMotor4; //
extern motor ToolMotor5;

extern digital_out digitalA;
extern digital_out digitalB;
extern digital_out digitalC;
extern digital_out digitalD;
extern digital_out digitalE; // 抬升
extern digital_out digitalF;
extern digital_out digitalH; // 抬升
extern digital_out digitalG;

extern inertial ChassisInertial; // 底盘陀螺仪
extern smartdrive Smartdrive;    // 官方底盘驱动

extern rotation Rotation_x; // 底盘定位轮x
extern rotation Rotation_y; // 底盘定位轮y

extern optical Optical1; // 颜色传感器
extern optical Optical2; // 颜色传感器

// 距离传感器
extern distance Distance_f;  // 前
extern distance Distance_b;  // 后
extern distance Distance_fl; // 左前
extern distance Distance_fr; // 右前
extern distance Distance_bl; // 左后
extern distance Distance_br; // 右后
extern distance Distance_l;  // 左
extern distance Distance_r;  // 右

extern ClawController clawController;

// 视觉传感器
// extern aivision::colordesc Red;
// extern aivision::colordesc Bule;
// extern aivision::colordesc Orange;
// extern aivision aiVision;

extern std::vector<AutoFun> AutoFunctions;                               // 自动路径储存
extern AutoFun *RunningAutoFun;                                          // 当前自动路径
extern std::vector<ChassisRomoteControlParams> ChassisRomoteControlList; // 遥控列表
extern ChassisRomoteControl ChassisRomoteControlSTS;                     // 遥控控制

extern std::vector<char *> MotorNames;
extern std::vector<motor> Motors;

extern std::vector<char *> DistanceNames;
extern std::vector<distance> Distances;

// 记忆模式相关
extern std::string FileName;

extern bool InertialIsReallyGood;    // 自动校准陀螺仪飘值是否成功
extern int InertialCalibrationCount; // 陀螺仪校准次数

#endif