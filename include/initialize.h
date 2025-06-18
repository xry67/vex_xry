#ifndef INITIALIZE_H  
#define INITIALIZE_H 

#include "vex.h"
#include "robot-config.h"
#include "units.h"
#include "auto.h"
#include "Positioning.h"

// 陀螺仪初始化
void ChassisInertialInit(){
    ChassisInertial.calibrate(5);  // 陀螺仪校准

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

// 电量监控
vex::task LowBatteryMonitor_task;
int LowBatteryMonitor(){
    while (true) {
        if (Brain.Battery.capacity(pct) <= PowerLimit) {
            Controller.rumble("..");
            wait(500, msec);
        }
        Brain.Screen.printAt( 200, 200, "Angle %f", ChassisInertial.rotation());
        vex::task::sleep(1000);
    }
}

// 初始化 
void vexcodeInit(void) {
    LowBatteryMonitor_task = vex::task(LowBatteryMonitor);  // 电量监控开启
    ChassisInertialInit();  // 底盘陀螺仪初始化
    AddAutoFun();           // 自动函数初始化
    // Flywheel.setVelocity(100, dps);
    // PositioningSTS.Reset(); // 底盘定位初始化
    // PositioningSTS.TurnOn(); // 底盘定位开启
    Optical.setLightPower(50, percent);
}

#endif 