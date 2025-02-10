#include "vex.h"
#include "robot-config.h"

using namespace vex;

int ControllerScreenWidth = 210;  // 屏幕宽度
int RemoteControlMod      = 0;    // 遥控模式  1：双 2：左 3：右 4：坦克
bool RemoteControlModLock = false ;// ture: 不再选择遥控模式，以RemoteControlMod初始值为准，false:需要选择
int PowerLimit            = 25;   // 电量低警示
double blindCA            = 2;    // 摇杆死区
char stance               = 'r';  // 站位选择

// 自动程序加入
std::vector<AutoFun> AutoFunctions;


//  硬件定义
brain Brain; 
controller Controller = controller();

motor LeftMotor1 = motor(PORT11, ratio6_1, 0);//11
motor LeftMotor2 = motor(PORT12, ratio6_1, 1);//12
motor LeftMotor3 = motor(PORT13, ratio6_1, 1);//13
motor_group LeftMotorGroup = motor_group(LeftMotor1, LeftMotor2, LeftMotor3);  // 左边电机组
 
motor RightMotor1 = motor(PORT18, ratio6_1, 1);//18
motor RightMotor2 = motor(PORT19, ratio6_1, 0);//20
motor RightMotor3 = motor(PORT20, ratio6_1, 0);//19
motor_group RightMotorGroup = motor_group(RightMotor1, RightMotor2, RightMotor3);  // 右边边电机组

inertial ChassisInertial = inertial(PORT6);  // 底盘陀螺仪