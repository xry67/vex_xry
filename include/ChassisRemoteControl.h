#ifndef CHASSISERMOTECONTROL_H  
#define CHASSISERMOTECONTROL_H

#include <string>
#include <vector>
#include <memory>
#include "robot-config.h"
#include "units.h"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>



// 遥控手柄数据存储
class RemoteControlData{
public:
    int axis1 = 0;
    int axis2 = 0;
    int axis3 = 0;
    int axis4 = 0;

    bool ButtonA = 0;
    bool ButtonB = 0;
    bool ButtonX = 0;
    bool ButtonY = 0;

    bool ButtonDown = 0;
    bool ButtonUp = 0;
    bool ButtonLeft = 0;
    bool ButtonRight = 0;

    bool ButtonL1 = 0;
    bool ButtonL2 = 0;
    bool ButtonR1 = 0;
    bool ButtonR2 = 0;

    double TimeWait = 0;

    RemoteControlData()
    {}

    RemoteControlData(int axis1_, int axis2_, int axis3_, int axis4_,
                      int ButtonA_, int ButtonB_, int ButtonX_, int ButtonY_,
                      int ButtonDown_, int ButtonUp_, int ButtonLeft_, int ButtonRight_, 
                      int ButtonL1_, int ButtonL2_, int ButtonR1_, int ButtonR2_, 
                      int TimeWait_)
    {        
        axis1 = axis1_;  
        axis2 = axis2_;  
        axis3 = axis3_;  
        axis4 = axis4_;  

        ButtonA = ButtonA_;  
        ButtonB = ButtonB_;  
        ButtonX = ButtonX_;  
        ButtonY = ButtonY_;  

        ButtonDown = ButtonDown_;  
        ButtonUp = ButtonUp_;  
        ButtonLeft = ButtonLeft_;  
        ButtonRight = ButtonRight_;  

        ButtonL1 = ButtonL1_;  
        ButtonL2 = ButtonL2_;  
        ButtonR1 = ButtonR1_;  
        ButtonR2 = ButtonR2_;  

        TimeWait = TimeWait_;  
    }  

    void UpData(RemoteControlData data){
        axis1 = data.axis1;  
        axis2 = data.axis2;  
        axis3 = data.axis3;  
        axis4 = data.axis4;  

        ButtonA = data.ButtonA;  
        ButtonB = data.ButtonB;  
        ButtonX = data.ButtonX;  
        ButtonY = data.ButtonY;  

        ButtonDown = data.ButtonDown;  
        ButtonUp = data.ButtonUp;  
        ButtonLeft = data.ButtonLeft;  
        ButtonRight = data.ButtonRight;  

        ButtonL1 = data.ButtonL1;  
        ButtonL2 = data.ButtonL2;  
        ButtonR1 = data.ButtonR1;  
        ButtonR2 = data.ButtonR2;  

        TimeWait = data.TimeWait;
    }

    void save(std::ofstream &outFile){
        outFile << axis1 << " " 
                << axis2 << " " 
                << axis3 << " " 
                << axis4 << " " 
                << ButtonA << " " 
                << ButtonB << " " 
                << ButtonX << " " 
                << ButtonY << " " 
                << ButtonDown << " "
                << ButtonUp << " "
                << ButtonLeft << " "
                << ButtonRight << " "
                << ButtonL1 << " "
                << ButtonL2 << " "
                << ButtonR1 << " "
                << ButtonR2 << " "
                << TimeWait << " "
      << std::endl;
    }   

    void input(vex::controller &Controller, double TimeWait_) {
        axis1 = Controller.Axis1.position();
        axis2 = Controller.Axis2.position();
        axis3 = Controller.Axis3.position();
        axis4 = Controller.Axis4.position();

        ButtonA = Controller.ButtonA.pressing();
        ButtonB = Controller.ButtonB.pressing();
        ButtonX = Controller.ButtonX.pressing();
        ButtonY = Controller.ButtonY.pressing();

        ButtonDown = Controller.ButtonDown.pressing();
        ButtonUp = Controller.ButtonUp.pressing();
        ButtonLeft = Controller.ButtonLeft.pressing();
        ButtonRight = Controller.ButtonRight.pressing();

        ButtonL1 = Controller.ButtonL1.pressing();
        ButtonL2 = Controller.ButtonL2.pressing();
        ButtonR1 = Controller.ButtonR1.pressing();
        ButtonR2 = Controller.ButtonR2.pressing();

        // 更新 TimeWait 可以根据需要设置
        TimeWait = TimeWait_; // 或者根据实际情况更新
    }

    void print(){
      //std::cout << axis1 << " " 
    //             << axis2 << " " 
    //             << axis3 << " " 
    //             << axis4 << " " 
    //             << ButtonA << " " 
    //             << ButtonB << " " 
    //             << ButtonX << " " 
    //             << ButtonY << " " 
    //             << ButtonDown << " "
    //             << ButtonUp << " "
    //             << ButtonLeft << " "
    //             << ButtonRight << " "
    //             << ButtonL1 << " "
    //             << ButtonL2 << " "
    //             << ButtonR1 << " "
    //             << ButtonR2 << " "
    //             << TimeWait << " "
    //   << std::endl;
    }

    bool load(std::ifstream& inFile) {
        std::string line;
        if (std::getline(inFile, line)) {
            std::istringstream iss(line);
            iss >> axis1 >> axis2 >> axis3 >> axis4
                >> ButtonA >> ButtonB >> ButtonX >> ButtonY
                >> ButtonDown >> ButtonUp >> ButtonLeft >> ButtonRight
                >> ButtonL1 >> ButtonL2 >> ButtonR1 >> ButtonR2
                >> TimeWait;
            return true;
        }
        else{
            return false;
        }
    }

};

// 底盘遥控控制类
class ChassisRomoteControl{
    public:
    char* name = "未初始化";
    int MoveCount = 0;               // 移动气动检测
    void (*func)();  

    void SetRomoteControl(char* name_, void (*func_)()){
        name = name_;
        func = func_;
    }   

    void Work(){
        func();  // 运行指定函数
    }
    
};

// 底盘遥控控制基类
struct ChassisRomoteControlParams{
    char* name = "未命名遥控";  // 名称
    void (*Work)();                  // 控制函数

    // 构造函数
    ChassisRomoteControlParams(char* name_, void (*func_)())
    :name(name_), Work(func_){}
};

// 左电机控制
void LeftMotorControl(int val);

// 右电机控制
void RightMotorControl(int val);

// 电机停止
void BaseMotorStop(int mode);

// 双摇杆
void ChassisBothHandsRomoteControl();

// 左单
void ChassisLeftHandsRomoteControl();

// 右单
void ChassisRightHandsRomoteControl();

// 坦克
void ChassisTankRomoteControl();

// 人机遥控相关参数
class ChassisHumanMachineRomoteControlPara{
    public:
        double maxinum_speed_acceleration = 1;  // 最大加速度
        double last_speed = 0;
    
    double GetSpeed(double now_speed){
        double ans = 0;

        if (std::abs(now_speed) < 3){
            last_speed = 0;
            return ans;
        }

        if ((last_speed > 0 && now_speed < 0) || (last_speed < 0 && now_speed > 0) ){
            last_speed = 0;
        }

        if (last_speed > now_speed){
            ans = std::max(now_speed, last_speed - maxinum_speed_acceleration);
        }else{
            ans = std::min(now_speed, last_speed + maxinum_speed_acceleration);
        }
        //std::cout << now_speed << " " << last_speed << std::endl;
        last_speed = ans;
        // nowspeed = output;
        //std::cout << ans << std::endl;
        return ans;
    }
};

extern ChassisHumanMachineRomoteControlPara CHMRCP;

// 人机遥控
void ChassisHumanMachineRomoteControl();

// 加入底盘控制函数
void AddRomoteFun();

extern RemoteControlData ControlData;  // 记录遥控数据

#endif