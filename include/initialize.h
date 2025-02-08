#ifndef INITIALIZE_H  
#define INITIALIZE_H 

#include "vex.h"
#include "robot-config.h"
#include "units.h"
#include "auto.h"

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
    ChassisInertialInit();  // 底盘陀螺仪初始化
    AddAutoFun();           // 自动函数初始化
}
#endif 