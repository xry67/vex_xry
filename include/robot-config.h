#ifndef ROBOT_CONFIG_H  
#define ROBOT_CONFIG_H 

#include <vector>  
#include <functional>
#include "vex.h"
#include "AutoFun.h"


using namespace vex;

extern int ControllerScreenWidth;  // 屏幕宽度
extern int RemoteControlMod;       // 遥控模式


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

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);

extern std::vector<AutoFun> AutoFunctions;
void auto1();
void auto2();

#endif