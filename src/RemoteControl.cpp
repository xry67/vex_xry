#include "vex.h"
#include "robot-config.h"
#include "RemoteControl.h"
#include "ChassisControl.h"
#include "units.h"
#include <sstream>

// AiTarget ai_target =  AiTarget();

// 长按B检测
bool HoldButtonB(int count, int limit = 100)
{
    // 个数检测
    if (count >= limit)
    {
        return true;
    }

    PrintInLine(3);
    // Controller.Screen.print("             退出(B: %2d)", count);

    // 震动
    if (count == limit * 0.9)
    {
        Controller.rumble("..");
    }
    return false;
}

// 切换站位
bool ChangeStance()
{
    if (ControlData.ButtonR1 && ControlData.ButtonR2 && (ControlData.axis1 + ControlData.axis2 + ControlData.axis3 + ControlData.axis4) < 20)
    {
        stance = 'r';
        PrintInLine(1);
        Controller.Screen.print("遥控模式-Red  ");
        Controller.rumble(".");
    }
    if (ControlData.ButtonL1 && ControlData.ButtonL2 && (ControlData.axis1 + ControlData.axis2 + ControlData.axis3 + ControlData.axis4) < 20)
    {
        stance = 'b';
        PrintInLine(1);
        Controller.Screen.print("遥控模式-Blue");
        Controller.rumble("-");
    }
    return false;
}

int RemoteControl_child()
{ 
    bool LastB = false;
    while (1)
    {


        LastB = ControlData.ButtonB;
        wait(10, msec);
    }

    return 0;
}

void RemoteControl(bool SaveFile, bool ReadFile, int OutTime, std::string file_name, double timeout)
{

    PrintInLine(1);
    if (stance == 'r')
    {
        PrintInLine(1);
        Controller.Screen.print("遥控模式-Red");
    }
    else
    {
        PrintInLine(1);
        Controller.Screen.print("遥控模式-Blue");
    }

    std::ifstream inFile;
    std::ofstream outFile;

    if (ReadFile)
    {
        inFile.open(file_name);
    }

    if (SaveFile)
    {
        outFile.open(file_name);
    }

    // ai_target.SetColor(Red);

    PrintInLine(2);

    // 分球暂停计数器
    int ErrorBall1 = 0;
    int ErrorBall2 = 0;

    // 退出计数
    int HoldButtonBCount = 0;

    // 递球卡死检测
    int MotorBlocked = true;

    // 时间变量
    double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    double now_time = start_time;
    double out_time = 0;
    double last_time = start_time;

    bool LastX = false;
    bool LastB = false;
    bool LastL1 = false;
    bool LastL2 = false;

    vex::task Task(RemoteControl_child); // 线程开启

    while (now_time - start_time < timeout)
    {
        last_time = now_time;
        now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

        // 输入更新
        if (ReadFile)
        {
            if (!ControlData.load(inFile))
            {
                std::cout << "end" << std::endl;
                break;
            }
            ControlData.print();
        }
        else
        {
            ControlData.input(Controller, now_time - last_time);
        }

        // 如果需要保存
        if (SaveFile)
        {
            ControlData.save(outFile);
        }

        // 底盘控制
        ChassisRomoteControlSTS.Work();
        ChangeStance();

        // intake
        if (ControlData.ButtonR1){
          ToolMotor1.spin(forward, 13, volt);
          ToolMotor2.spin(forward, 13, volt);
        }else{
          ToolMotor1.spin(forward, 0, volt);
          ToolMotor2.spin(forward, 0, volt);
        }

        // 爪子
        if (LastL1 && !ControlData.ButtonL1)
        {
            clawController.changeMod(0);
        }
        if (LastL2 && !ControlData.ButtonL2)
        {
            clawController.changeMod(2);
        }

        // 反转
        if (LastB && !ControlData.ButtonB)
        {
            clawController.reversal();
        }

        // 抬升
        if (ControlData.axis2 > 5 || ControlData.axis2 < -5)
        {
            ToolMotor5.spin(forward, -ControlData.axis2, pct);
        }
        else
        {
            ToolMotor5.spin(forward, 0, pct);
        }

        if (!LastX && ControlData.ButtonX)
        {
            digitalH.set(!digitalH.value());
        }

        // 退出
        if (!Competition.isFieldControl() && ControlData.ButtonB)
        {
            HoldButtonBCount += 1;
            if (HoldButtonB(HoldButtonBCount, OutTime))
            {
                break;
            }
        }
        else
        {
            HoldButtonBCount = 0;
        }

        out_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

        if (ReadFile)
        {
            wait(std::max(0.0, ControlData.TimeWait - (out_time - now_time)), seconds);
        }
        else
        {
            wait(50, msec);
        }


        LastX = ControlData.ButtonX;
        LastB = ControlData.ButtonB;
        LastL1 = ControlData.ButtonL1;
        LastL2 = ControlData.ButtonL2;
    }
}

// 整体遥控
void RemoteControlUnit(bool SaveFile, bool ReadFile, int OutTime, std::string file_name, double timeout)
{

    RemoteControl(SaveFile, ReadFile, OutTime, file_name, timeout); // 三代车，双颜色传感器
}

// 整体遥控(手动)
void RemoteControl()
{
    RemoteControlUnit(false, false, 50, "", 999999999999);
}

// 整体遥控（自动）
void AutoRemoteControl(std::string file_name, int timeout)
{
    CHMRCP.last_speed = 0;
    RemoteControlUnit(false, true, 2, file_name, timeout);
}
