#ifndef REMOTECONTROL_H  
#define REMOTECONTROL_H

#include "vex.h"
#include "robot-config.h"

// 遥控手柄数据存储
class RemoteControlData{
public:
    int axis1 = 0;
    int axis2 = 0;
    int axis3 = 0;
    int axis4 = 0;

    int ButtonA = 0;
    int ButtonB = 0;
    int ButtonX = 0;
    int ButtonY = 0;

    int ButtonDown = 0;
    int ButtonUp = 0;
    int ButtonLeft = 0;
    int ButtonRight = 0;

    int ButtonL1 = 0;
    int ButtonL2 = 0;
    int ButtonR1 = 0;
    int ButtonR2 = 0;

    int TimeWait = 0;

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

    void print(){
      std::cout << axis1 << " " 
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

};

// 定长队列
class RemoteControlDataList{
    int len = 0;
    std::vector<RemoteControlData> list;
    int index = -1;

    public:
        RemoteControlDataList(int len_){
            // 填满整个队列
            len = len_;
            for (int i=0; i<len_; i++){
                list.push_back(RemoteControlData());
            }
        }

        // 入队
        void push(RemoteControlData data){
            index = (index + 1) % len;
            list[index].UpData(data); 
        }

        // 获得长度
        int size(){
          return len;
        }

        // 获取下标对应的数据
        RemoteControlData get(int val){
            return list[(index - val) % len];
        }

        // 获得最新的
        RemoteControlData get_new(){
            return list[index];
        }
};

// 底盘控制
void ChassisControl();

// 其他控制
void RemoteControl();

#endif


