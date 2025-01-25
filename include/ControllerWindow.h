#ifndef CONTROLLERWINDOW_H   
#define CONTROLLERWINDOW_H  

#include "vex.h"
#include <vector>

// 遥控器输出重定义到某一行
void PrintInLine(int line);

// 从选项中选择一个选项
int ChoiceIndex(std::vector<char*> options);

// 窗口基类
class BaseWindow{
    public: 
    BaseWindow(std::string name_, BaseWindow* DadWindow_, BaseWindow* (*funcPtr_)()){
        name = name_;
        DadWindow = DadWindow_;
        funcPtr = funcPtr_;
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


// Controller.Screen.clearScreen();  // 清除屏幕

#endif