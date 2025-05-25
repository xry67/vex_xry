#include "vex.h"
#include "robot-config.h"

using namespace vex;

int ControllerScreenWidth = 210;  // 屏幕宽度
int RemoteControlMod      = 0;    // 遥控模式  1：双 2：左 3：右 4：坦克
bool RemoteControlModLock = false ;// ture: 不再选择遥控模式，以RemoteControlMod初始值为准，false:需要选择
int PowerLimit            = 25;   // 电量低警示
double blindCA            = 2;    // 摇杆死区
char stance               = 'r';  // 站位选择
int aiVision_w            = 316;  // ai相机的宽度 
int aiVision_h            = 211;  // ai相机的高度

double Sensitivity_Turn       = 0.9;  // 转向灵敏度
double Sensitivity_Straight   = 1;    // 直行灵敏度
double Sensitivity_Precision  = 0.1;  // 精密遥控比例

// 自动程序加入
std::vector<AutoFun> AutoFunctions;


//  硬件定义
brain Brain; 
controller Controller = controller();

motor LeftMotor1 = motor(PORT8, ratio6_1, 0);//11
motor LeftMotor2 = motor(PORT9, ratio6_1, 1);//12
motor LeftMotor3 = motor(PORT10, ratio6_1, 1);//13
motor_group LeftMotorGroup = motor_group(LeftMotor1, LeftMotor2, LeftMotor3);  // 左边电机组
 
motor RightMotor1 = motor(PORT18, ratio6_1, 1);//18
motor RightMotor2 = motor(PORT19, ratio6_1, 0);//20
motor RightMotor3 = motor(PORT20, ratio6_1, 0);//19
motor_group RightMotorGroup = motor_group(RightMotor1, RightMotor2, RightMotor3);  // 右边边电机组

inertial ChassisInertial = inertial(PORT16);  // 底盘陀螺仪

rotation Rotation_x = rotation(PORT14, true);  // 底盘定位轮x
rotation Rotation_y = rotation(PORT1, false);  // 底盘定位轮y

// 视觉传感器
aivision::colordesc Red = aivision::colordesc(1, 219, 21, 65, 10.00, 0.2);
aivision aiVision = aivision(PORT13, Red);

// 距离传感器
distance Distance_fl = distance(PORT2);  // 左前
distance Distance_fr = distance(PORT12);  // 右前
distance Distance_bl = distance(PORT7);  // 左后
distance Distance_br = distance(PORT17);  // 右后
distance Distance_l = distance(PORT1);  // 左后
distance Distance_r = distance(PORT11);  // 右后