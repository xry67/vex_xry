#ifndef INITIALIZE_H
#define INITIALIZE_H

#include "vex.h"
#include "robot-config.h"
#include "units.h"
#include "auto.h"
#include "ChassisRemoteControl.h"
#include "RobotControl.h"
#include "draw.h"

// 陀螺仪初始化
void ChassisInertialInit()
{
    ChassisInertial.calibrate(); // 陀螺仪校准
    vex::task Task(InertialCalibrating);
}

// 设备监控
vex::task EquipmentMonitor_task;
int EquipmentMonitor()
{
    DrawEquipmentMonitor Draw_Equipment_Monitor = DrawEquipmentMonitor();
    Draw_Equipment_Monitor.StartInit();
    while (true)
    {
        // 电量监控
        if (Brain.Battery.capacity(pct) <= PowerLimit)
        {
            Controller.rumble("..");
            wait(500, msec);
        }

        // Brain.Screen.printAt(100, 150, "%d %d %d", Competition.isCompetitionSwitch(), Competition.isEnabled(), Competition.isFieldControl());

        // 设备监控
        Draw_Equipment_Monitor.Updata();

        Brain.Screen.printAt(280, 80, "Lift %f", Rotation.angle());
        Brain.Screen.printAt(280, 100, "Angle %f", ChassisInertial.rotation());
        vex::task::sleep(1000);
    }
}

// 等待按下x
void WaitForX()
{
    PrintInLine(3);
    Controller.Screen.print("               忽略(X)");
    while (true)
    {
        if (Controller.ButtonX.pressing())
        {
            return;
        }
        wait(10, msec);
    }
}

// 寻找陀螺仪
bool FindChassisInertial()
{
    for (int32_t now_port = 0; now_port < 21; now_port++)
    {
        ChassisInertial = inertial(now_port); // 底盘陀螺仪
        if (ChassisInertial.installed())
        {
            return true;
        }
    }
    return false;
}

// 爪子状态维护
vex::task clawStateMaintain_task;
int clawStateMaintain()
{
    double reversal_p = 0.12;
    while (true)
    {
        Claw.maintenance();

        wait(10, msec);
    }
    return 0;
}

// 设备检查
void CheckEquipment()
{
    // 电机检查
    PrintInLine(1);
    Controller.Screen.print("设备检查中...");

    // 电机检查
    for (int i = 0; i < Motors.size(); i++)
    {
        if (!Motors[i].installed())
        {
            PrintInLine(2);
            Controller.Screen.print("%s电机失联", MotorNames[i]);
            WaitForX();
        }
    }

    // 陀螺仪检查
    if (!FindChassisInertial())
    {
        PrintInLine(2);
        Controller.Screen.print("陀螺仪失联");
        WaitForX();
    }

    // 其他检查
}

// 夹子初始化
void ClawControllerInit()
{
    PrintInLine(2);
    Controller.Screen.print("按(A)开始限位设置喵");
    PrintInLine(3);
    Controller.Screen.print("注意 抬升会动喵");
    wait(200, msec);
    while(!Controller.ButtonA.pressing()) wait(10, msec);;
    Claw.init();
}

// 初始化
void vexcodeInit(void)
{
    CheckEquipment();                                      // 设备检查
    EquipmentMonitor_task = vex::task(EquipmentMonitor);   // 电量监控开启
    ClawControllerInit();                                  // 夹子初始化
    clawStateMaintain_task = vex::task(clawStateMaintain); // 爪子状态维护开启

    // Claw.changeMod(1);
	// wait(500, msec);
    //Claw.modMove();
	//wait(200, msec);
    Claw.LiftMove_Normalized(0.25);
	wait(200, msec);

    // 初始位置设置
    // ClawControllerInit();
    ChassisInertialInit(); // 底盘陀螺仪初始化

    ToolMotor1.resetPosition();
    ToolMotor2.resetPosition();

    Optical1.setLightPower(100, percent);
    Optical2.setLightPower(100, percent);

    AddAutoFun();   // 自动函数初始化
    AddRomoteFun(); // 遥控函数初始化
}

#endif