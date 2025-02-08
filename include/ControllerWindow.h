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
        std::cout << name << std::endl;
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
extern BaseWindow* AutoWindow;  //  自动测试模式
extern BaseWindow* TestWindow;  //  测试模式

extern BaseWindow* AutoChoiceWindow_Compete;     //  比赛-自动选择
extern BaseWindow* RemoteControlWindow_Auto;  // 自动-遥控选择
extern BaseWindow* TestMotorChoiceWindow;        // 电机测试（选择）
extern BaseWindow* TestAirpoweredChoiceWindow;   // 活塞测试（选择）
extern BaseWindow* TestInertialChoiceWindow;     // 陀螺仪测试（选择）

extern BaseWindow* TestMotorWindow;        // 电机测试
extern BaseWindow* TestAirpoweredWindow;   // 活塞测试
extern BaseWindow* TestInertialWindow;     // 陀螺仪测试

// 各个函数实现------------------------------------------

// 初始化函数
BaseWindow* InitWindowFun();

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


// 电机测试
BaseWindow* TestMotorWindowFun();        

// 活塞测试
BaseWindow* TestAirpoweredWindowFun();   

// 陀螺仪测试
BaseWindow* TestInertialWindowFun();     


// Controller.Screen.clearScreen();  // 清除屏幕

#endif