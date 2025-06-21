#include "vex.h"
#include "robot-config.h"

using namespace vex;

int ControllerScreenWidth = 210;  // 屏幕宽度
int RemoteControlMod      = 0;    // 遥控模式  1：双 2：左 3：右 4：坦克
bool RemoteControlModLock = false ;// ture: 不再选择遥控模式，以RemoteControlMod初始值为准，false:需要选择
int PowerLimit            = 25;   // 电量低警示
double blindCA            = 0;    // 摇杆死区
char stance               = 'r';  // 站位选择
int aiVision_w            = 316;  // ai相机的宽度 
int aiVision_h            = 211;  // ai相机的高度
double StraightCorrectionSpeed = 500;  // 执行修正pps最大速度

double Sensitivity_Turn       = 0.9;  // 转向灵敏度
double Sensitivity_Straight   = 1;    // 直行灵敏度
double Sensitivity_Precision  = 0.1;  // 精密遥控比例

double TrackSpeed   = 300000000;  // 履带速度

// 遥控期间自动追踪的灵敏度
double Auto_Turn_To_Target_Speed = 50;

// 自动程序加入
std::vector<AutoFun> AutoFunctions;


//  硬件定义
brain Brain; 
controller Controller = controller();

motor LeftMotor1 = motor(PORT11, ratio6_1, 0);//11
motor LeftMotor2 = motor(PORT12, ratio6_1, 1);//12
motor LeftMotor3 = motor(PORT15, ratio6_1, 1);//13
motor_group LeftMotorGroup = motor_group(LeftMotor1, LeftMotor2, LeftMotor3);  // 左边电机组
 
motor RightMotor1 = motor(PORT16, ratio6_1, 1);//18
motor RightMotor2 = motor(PORT17, ratio6_1, 0);//20
motor RightMotor3 = motor(PORT18, ratio6_1, 0);//19
motor_group RightMotorGroup = motor_group(RightMotor1, RightMotor2, RightMotor3);  // 右边边电机组

motor ToolMotor1    = motor(PORT3, ratio6_1, 1);  // 上  f
motor ToolMotor2    = motor(PORT2, ratio6_1, 0);  // 中  l
motor ToolMotor3    = motor(PORT1, ratio6_1, 1);  // 下  r
motor ToolMotor4    = motor(PORT4, ratio6_1, 0);  // 

digital_out digitalA = digital_out(Brain.ThreeWirePort.A);  // 抬升
digital_out digitalB = digital_out(Brain.ThreeWirePort.B);  // 抬升

optical Optical = optical(PORT10);  // 颜色传感器

inertial ChassisInertial = inertial(PORT21);  // 底盘陀螺仪

rotation Rotation_x = rotation(PORT21, true);  // 底盘定位轮x
rotation Rotation_y = rotation(PORT21, false); // 底盘定位轮y

// 视觉传感器
aivision::colordesc Red = aivision::colordesc(1, 219, 21, 65, 10.00, 0.2);
aivision::colordesc Bule = aivision::colordesc(2, 65, 160, 213, 12.00, 0.36);
aivision::colordesc Orange = aivision::colordesc(3, 198, 102, 6, 15.00, 0.14);
aivision aiVision = aivision(PORT21, Red, Bule, Orange);

// 距离传感器
distance Distance_fl = distance(PORT21);  // 左前
distance Distance_fr = distance(PORT21);  // 右前
distance Distance_bl = distance(PORT21);  // 左后
distance Distance_br = distance(PORT21);  // 右后
distance Distance_l  = distance(PORT21);  // 左后
distance Distance_r  = distance(PORT21);  // 右后

// 设别列表
std::vector<char*> MotorNames = {"L1", "L2", "L3", "R1", "R2", "R3"};
std::vector<motor> Motors = {LeftMotor1, LeftMotor2, LeftMotor3, 
                                RightMotor1, RightMotor2, RightMotor3};

std::vector<char*> DistanceNames = {"Fl", "Fr", "Bl", "Nr", "L", "R"};
std::vector<distance> Distances = {Distance_fl, Distance_fr, Distance_bl, 
                                Distance_br, Distance_l, Distance_r};

// 设备检测绘图