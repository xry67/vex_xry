#ifndef INITIALIZE_H  
#define INITIALIZE_H 

#include "vex.h"
#include "robot-config.h"
#include "units.h"
#include "auto.h"
#include "Positioning.h"
#include "draw.h"

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

// 设备监控
vex::task EquipmentMonitor_task;
int EquipmentMonitor(){
    DrawEquipmentMonitor Draw_Equipment_Monitor = DrawEquipmentMonitor();
    Draw_Equipment_Monitor.StartInit();
    while (true) {
        // 电量监控
        if (Brain.Battery.capacity(pct) <= PowerLimit) {
            Controller.rumble("..");
            wait(500, msec);
        }

        // 设备监控
        Draw_Equipment_Monitor.Updata();

        Brain.Screen.printAt(280, 100, "Angle %f", ChassisInertial.rotation());
        vex::task::sleep(1000);
    }
}

// 等待按下x
void WaitForX(){
    PrintInLine(3);
    Controller.Screen.print("               忽略(X)");
    while(true){
        if (Controller.ButtonX.pressing()){
            return ;
        }
        wait(10, msec);
    }
}

// 设备检查
void CheckEquipment(){
    // 电机检查
    PrintInLine(1);
    Controller.Screen.print("设备检查中...");

    // 电机检查
    for (int i=0; i < Motors.size(); i++){
        if (! Motors[i].installed()){
            PrintInLine(2);
            Controller.Screen.print("%s电机失联", MotorNames[i]);
            WaitForX();
        }
    }

    // 陀螺仪检查
    if (! ChassisInertial.installed()){
        PrintInLine(2);
        Controller.Screen.print("陀螺仪失联");
        WaitForX();
    }

    // 其他检查
}

// 初始化 
void vexcodeInit(void) {
    CheckEquipment();  // 设备检查
    EquipmentMonitor_task = vex::task(EquipmentMonitor);  // 电量监控开启
    ChassisInertialInit();  // 底盘陀螺仪初始化
    AddAutoFun();           // 自动函数初始化
}

#endif 