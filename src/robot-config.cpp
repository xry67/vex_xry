#include "vex.h"
#include "AutoFun.h"

using namespace vex;

int ControllerScreenWidth = 210;  // 屏幕宽度
int RemoteControlMod      = 0;    // 遥控模式  1：左 2：右 3：双 4：坦克


// A global instance of brain used for printing to the V5 brain screen
std::vector<AutoFun> AutoFunctions = {AutoFun("自动程序1", auto1), 
                                      AutoFun("自动程序2", auto2)};


//  硬件定义
brain Brain; 
controller Controller = controller();

motor LeftMotor1 = motor(PORT14, ratio6_1,1);//14
motor LeftMotor2 = motor(PORT15, ratio6_1,1);//15
motor LeftMotor3 = motor(PORT16, ratio6_1, 1);//16
motor_group LeftMotorGroup = motor_group(LeftMotor1, LeftMotor2, LeftMotor3);  // 左边电机组
 
motor RightMotor1 = motor(PORT18, ratio6_1, 0);//18
motor RightMotor2 = motor(PORT19, ratio6_1, 0);//20
motor RightMotor3 = motor(PORT20, ratio6_1,0);//19
motor_group RightMotorGroup = motor_group(RightMotor1, RightMotor2, RightMotor3);  // 右边边电机组

inertial ChassisInertial = inertial(PORT6);  // 底盘陀螺仪

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */

// 陀螺仪初始化
void ChassisInertialInit(){
    ChassisInertial.calibrate();  // 陀螺仪校准

    Controller.Screen.clearScreen();  // 清除屏幕

    WaitStr wait_str = WaitStr();
  
    while(ChassisInertial.isCalibrating()){
        wait(500, msec);

        PrintInLine(2);
        char formattedString[50]; // 创建字符数组来存储格式化字符串  
        // 使用 snprintf 生成格式化字符串  
        snprintf(formattedString, 50, "-> 初始化中 %s <-", wait_str.GetNextStr()); 
        // 使字符串居中
        CenterStr(formattedString);

        Controller.Screen.print(formattedString);
    }
}

// 初始化 
void vexcodeInit(void) {
  ChassisInertialInit();
}