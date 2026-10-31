#ifndef CONTROLLERWINDOW_H   
#define CONTROLLERWINDOW_H  

#include "vex.h"
#include "robot-config.h"
#include <vector>

// 从选项中选择一个选项
int ChoiceIndex(std::vector<char*> options);

// 窗口基类
class BaseWindow{
    public: 
    int num = 0;
    BaseWindow(std::string name_, BaseWindow* DadWindow_, BaseWindow* (*funcPtr_)()){
        name = name_;
        DadWindow = DadWindow_;
        funcPtr = funcPtr_;
        num = 0;
    }

    BaseWindow* Tick(){
        Controller.Screen.clearScreen();  // 清除屏幕
        //std::cout << name << std::endl;
        return funcPtr();
    }

    BaseWindow* Back(){
        return DadWindow;
    }

    private: 
    std::string name;
    BaseWindow* DadWindow;
    BaseWindow* (*funcPtr)();
};

// 主窗口类
class MainWindow{
    public: 
    void Run();
};

// 各个窗口命名------------------------------------------
extern BaseWindow* InitWindow; //  初始化窗口
extern BaseWindow* ModWindow;  //  模式选择窗口

extern BaseWindow* RemoteControlWindow_Mod;      //  模式-遥控
extern BaseWindow* RemoteControlWindow_Compete;  //  比赛-遥控模式
extern BaseWindow* AutoWindow;   //  自动测试模式
extern BaseWindow* TestWindow;   //  测试模式
extern BaseWindow* MemoryWindow; //  记忆模式

extern BaseWindow* AutoChoiceWindow_Compete;     //  比赛-自动选择
extern BaseWindow* RemoteControlWindow_Auto;  // 自动-遥控选择
extern BaseWindow* TestMotorChoiceWindow;        // 电机测试（选择）
extern BaseWindow* TestAirpoweredChoiceWindow;   // 活塞测试（选择）
extern BaseWindow* TestInertialChoiceWindow;     // 陀螺仪测试（选择）
extern BaseWindow* TestDistanceChoiceWindow;     // 距离传感器测试（选择）
extern BaseWindow* TestDecenteringWindow;        // 电机脱轴测试

extern BaseWindow* TestMotorWindow;        // 电机测试
extern BaseWindow* TestAirpoweredWindow;   // 活塞测试
extern BaseWindow* TestInertialWindow;     // 陀螺仪测试
extern BaseWindow* TestDistanceWindow;     // 距离传感器测试
extern BaseWindow* TestRotationWindow;     // 定位轮测试

// 各个函数实现------------------------------------------

// 初始化函数
BaseWindow* InitWindowFun();

// 遥控模式选择
BaseWindow* RemoteControlWindowFun();

// 模式选择
BaseWindow* ModWindowFun();

//  模式-遥控
BaseWindow* RemoteControlWindow_ModFun();     

//  比赛-遥控模式
BaseWindow* RemoteControlWindow_CompeteFun();  

//  自动测试模式
BaseWindow* AutoWindowFun();  

//  测试模式
BaseWindow* TestWindowFun();

// 记忆模式
BaseWindow* MemoryWindowFun();

//  比赛-自动选择
BaseWindow* AutoChoiceWindow_CompeteFun();     
 

// 自动-遥控选择
BaseWindow* RemoteControlWindow_AutoFun();  

// 电机测试（选择）
BaseWindow* TestMotorChoiceWindowFun();    

// 活塞测试（选择）
BaseWindow* TestAirpoweredChoiceWindowFun();   

// 陀螺仪测试（选择）
BaseWindow* TestInertialChoiceWindowFun();     

// 距离传感器测试（选择）
BaseWindow* TestDistanceChoiceWindowFun();   

// 脱轴检测
BaseWindow* TestDecenteringFun();

// 比赛确认选择
BaseWindow* CompeteConfirmWindowFun();

// 比赛-连接场控
BaseWindow* CompetitionlWindowFun();

// 电机测试
BaseWindow* TestMotorWindowFun();        

// 活塞测试
BaseWindow* TestAirpoweredWindowFun();   

// 陀螺仪测试
BaseWindow* TestInertialWindowFun();   

// 距离传感器测试
BaseWindow* TestDistanceWindowFun();   

// 定位轮测试
BaseWindow* TestRotationWindowFun();
// Controller.Screen.clearScreen();  // 清除屏幕

#endif