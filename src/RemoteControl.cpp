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
    bool LastRight = false;
    while (1)
    {

        // if (!ControlData.ButtonRight && LastRight)
        // {
        //     // 关闭夹子
        //     digitalA.set(1);
        //     wait(500, msec);
        //     digitalC.set(0);
        //     wait(500, msec);

        //     // 电梯下降
        //     int count = 0;
        //     while (true)
        //     {
        //         // 按键中断
        //         if (ControlData.ButtonL1 || ControlData.ButtonL2 || ControlData.ButtonB || ControlData.ButtonX)
        //         {
        //             break;
        //         }

        //         ToolMotor1.spin(forward, 13, volt);
        //         ToolMotor2.spin(forward, 13, volt);

        //         // std::cout << "-->" << reversal_motor.velocity(dps) << std::endl;
        //         if (abs(ToolMotor1.velocity(dps)) <= 2 || abs(ToolMotor2.velocity(dps)) <= 2)
        //         {
        //             count += 1;
        //         }
        //         else
        //         {
        //             count = 0;
        //         }
        //         if (count >= 5)
        //         {
        //             break;
        //         }
        //         wait(10, msec);
        //     }

        //     // 电机停转
        //     ToolMotor1.spin(forward, 0, volt);
        //     ToolMotor2.spin(forward, 0, volt);
        // }

        LastRight = ControlData.ButtonRight;
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

        // 底盘控制
        ChassisRomoteControlSTS.Work();
        ChangeStance();

        // intake
        if (ControlData.ButtonR1)
        {
            ToolMotor3.spin(forward, 13, volt);
        }
        else if (ControlData.ButtonR2)
        {
            ToolMotor3.spin(forward, -13, volt);
        }
        else
        {
            ToolMotor3.spin(forward, 0, volt);
        }

        // 爪子 折叠
        if (LastDown && !ControlData.ButtonDown)
        {
            digitalC.set(!digitalC.value());
        }

        // 夹子 反转
        if (LastB && !ControlData.ButtonB)
        {
            digitalB.set(!digitalB.value());
        }

        // 夹子 开合
        if (!LastX && ControlData.ButtonX)
        {
            digitalA.set(!digitalA.value());
        }

        // 抬升
        std::cout << "-->" << ToolMotor1.torque(Nm) << std::endl;
        if (ControlData.ButtonL1)
        {
            ToolMotor1.spin(forward, -13, volt);
            ToolMotor2.spin(forward, -13, volt);
        }
        else if (ControlData.ButtonL2)
        {
            if (bumper_elevator.pressing())
            {
                // Controller.rumble("..");
                ToolMotor1.stop();
                ToolMotor1.setStopping(brake);
                ToolMotor2.stop();
                ToolMotor3.setStopping(brake);
            }
            else
            {
                ToolMotor1.spin(forward, 13, volt);
                ToolMotor2.spin(forward, 13, volt);
            }
        }
        else
        {
            ToolMotor1.stop();
            ToolMotor1.setStopping(brake);
            ToolMotor2.stop();
            ToolMotor3.setStopping(brake);
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
        LastDown = ControlData.ButtonDown;
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
