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
    while (1)
    {

        wait(10, msec);
    }

    return 0;
}

// 三代车，双颜色传感器
void RemoteControl(bool SaveFile, bool ReadFile, int OutTime, std::string file_name, double timeout)
{

    // digitalH  后方 起来
    // digitalG  中间 高桥
    // F         钩子 下来
    // E         铲子 收起来

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

    // 按钮上一状态记录
    bool LastRight = false;
    bool LastY = false;
    bool LastDown = false;

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

        // 分球
        if ((stance == 'r' && OpticalHaveBlue(Optical1)) || (stance == 'b' && OpticalHaveRed(Optical1)))
        {
            ErrorBall1 = 20;
        }
        else if ((stance == 'b' && OpticalHaveBlue(Optical1)) || (stance == 'r' && OpticalHaveRed(Optical1)))
        {
            ErrorBall1 = 0;
        }
        else
        {
            ErrorBall1 = -1;
        }

        if ((stance == 'r' && OpticalHaveBlue(Optical2)) || (stance == 'b' && OpticalHaveRed(Optical2)))
        {
            ErrorBall2 = 20;
        }
        else if ((stance == 'b' && OpticalHaveBlue(Optical2)) || (stance == 'r' && OpticalHaveRed(Optical2)))
        {
            ErrorBall2 = 0;
        }
        else
        {
            ErrorBall2 = -1;
        }

        // 底盘控制
        ChassisRomoteControlSTS.Work();

        ChangeStance();

        if (ControlData.ButtonDown)
        {
            digitalH.set(1);
        }

        if (!LastY && ControlData.ButtonY)
        {
            digitalE.set(!digitalE.value());
        }

        if (!LastRight && ControlData.ButtonRight)
        {
            digitalF.set(!digitalF.value());
        }

        // 变量维护
        LastRight = ControlData.ButtonRight;
        LastY = ControlData.ButtonY;
        LastDown = ControlData.ButtonDown;

        ErrorBall1 = std::max(0, ErrorBall1 - 1);
        ErrorBall2 = std::max(0, ErrorBall2 - 1);

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
