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

// 遥控额外线程控制 刘老师版本
int RemoteControl_child()
{
    return 0;
}

// 三代车，双颜色传感器
void RemoteControl(bool SaveFile, bool ReadFile, int OutTime, std::string file_name, double timeout)
{

    // digitalH  铲子
    // digitalG  钩子

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
    int MotorBlocked = false;

    // 时间变量
    double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    double now_time = start_time;
    double out_time = 0;
    double last_time = start_time;

    // 按钮上一状态记录
    bool LastRight = false;
    bool LastY = false;
    bool LastUp = false;

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

        // R1 吸球
        // R2 底层吐球
        // L1 高层吐球
        // L2 中层吐球
        // Y  铲子
        // 右 钩子

        // 1 号
        if (ControlData.ButtonR1 || ControlData.ButtonR2 || ControlData.ButtonL1 || ControlData.ButtonL2 || ControlData.ButtonDown)
        {
            if (ControlData.ButtonR1)
            {
                ToolMotor1.spin(forward, -16, voltageUnits::volt);
            }
            if (ControlData.ButtonR2)
            {
                ToolMotor1.spin(forward, 16, voltageUnits::volt);
            }
            if (ControlData.ButtonL1)
            {
                ToolMotor1.spin(forward, 16, voltageUnits::volt);
                MotorBlocked = false;
            }
            if (ControlData.ButtonL2)
            {
                ToolMotor1.spin(forward, -50, pct);
            }
            if (ControlData.ButtonDown)
            {
                ToolMotor1.spin(forward, -20, pct);
                MotorBlocked = true; // 取消递球
            }
        }
        else
        {
            if (ToolMotor1.torque() > 0.3 && std::abs(ToolMotor1.velocity(dps)) < 50)
            {
                MotorBlocked = true;
            }
            if (!MotorBlocked)
            {
                ToolMotor1.spin(forward, 2, voltageUnits::volt);
            }
            else
            {
                ToolMotor1.spin(forward, 0, voltageUnits::volt);
            }
        }

        // 2号
        if (ControlData.ButtonR1 || ControlData.ButtonR2 || ControlData.ButtonL1 || ControlData.ButtonL2 || ControlData.ButtonDown)
        {
            if (ControlData.ButtonR1)
            {
                //  分球逻辑
                // ErrorBall1： 靠前方的颜色传感器
                // ErrorBall2： 靠后方的颜色传感器
                // 0   ：正确颜色的球
                // -1  ：没有球
                // 1~20：错误的球，检测到错球值为20

                if (ErrorBall1 > 0 && ErrorBall2 > 0)
                {
                    // 前后都是异色球
                    ToolMotor2.spin(forward, 16, voltageUnits::volt);
                }
                else if (ErrorBall1 > 0 && ErrorBall2 < 0)
                {
                    // 前异色，后没球
                    ToolMotor2.spin(forward, 16, voltageUnits::volt);
                }
                else if (ErrorBall1 > 0 && ErrorBall2 == 0)
                {
                    // 前异色，后同色
                    ToolMotor2.spin(forward, -16, voltageUnits::volt);
                }
                else if (ErrorBall1 == 0 && ErrorBall2 > 0)
                {
                    // 前同色，后异色
                    ToolMotor2.spin(forward, 16, voltageUnits::volt);
                }
                else if (ErrorBall1 == 0 && ErrorBall2 == 0)
                {
                    // 前同色，后同色
                    ToolMotor2.spin(forward, -16, voltageUnits::volt);
                }
                else if (ErrorBall1 == 0 && ErrorBall2 < 0)
                {
                    // 前同色，后没球
                    ToolMotor2.spin(forward, -16, voltageUnits::volt);
                }
                else if (ErrorBall1 < 0 && ErrorBall2 > 0)
                {
                    // 前没球，后异色
                    ToolMotor2.spin(forward, 16, voltageUnits::volt);
                }
                else if (ErrorBall1 < 0 && ErrorBall2 == 0)
                {
                    // 前没球，后同色
                    ToolMotor2.spin(forward, -16, voltageUnits::volt);
                }
                else if (ErrorBall1 < 0 && ErrorBall2 < 0)
                {
                    // 前没球，后没球
                    ToolMotor2.spin(forward, -16, voltageUnits::volt);
                }
                else
                {
                    // 不可能进来，但还是留有保底
                    ToolMotor2.spin(forward, 16, voltageUnits::volt);
                }
            }
            if (ControlData.ButtonR2)
            {
                ToolMotor2.spin(forward, 40, pct);
            }
            if (ControlData.ButtonL1)
            {
                ToolMotor2.spin(forward, 0, voltageUnits::volt);
            }
            if (ControlData.ButtonL2)
            {
                ToolMotor2.spin(forward, 0, voltageUnits::volt);
            }
            if (ControlData.ButtonDown)
            {
                ToolMotor2.spin(forward, -16, voltageUnits::volt);
            }
        }
        else
        {
            if (ToolMotor2.torque() > 0.3 && std::abs(ToolMotor2.velocity(dps)) < 50)
            {
                MotorBlocked = true;
            }
            if (!MotorBlocked)
            {
                ToolMotor2.spin(forward, 5, pct);
            }
            else
            {
                ToolMotor2.spin(forward, 0, voltageUnits::volt);
            }
        }

        // 3号
        if (ControlData.ButtonR1 || ControlData.ButtonR2 || ControlData.ButtonL1 || ControlData.ButtonL2 || ControlData.ButtonDown)
        {
            if (ControlData.ButtonR1)
            {
                ToolMotor3.spin(forward, -10, voltageUnits::volt);
            }
            if (ControlData.ButtonR2)
            {
                ToolMotor3.spin(forward, 2, voltageUnits::volt);
            }
            if (ControlData.ButtonL1)
            {
                ToolMotor3.spin(forward, 16, voltageUnits::volt);
            }
            if (ControlData.ButtonL2)
            {
                ToolMotor3.spin(forward, 16, voltageUnits::volt);
            }
            if (ControlData.ButtonDown)
            {
                ToolMotor3.spin(forward, -10, voltageUnits::volt);
            }
        }
        else
        {
            if (ToolMotor3.torque() > 0.3 && std::abs(ToolMotor3.velocity(dps)) < 50)
            {
                MotorBlocked = true;
            }
            if (!MotorBlocked)
            {
                ToolMotor3.spin(forward, 30, pct);
            }
            else
            {
                ToolMotor3.spin(forward, 0, voltageUnits::volt);
            }
        }

        // 铲子钩子（新增中桥消分）
        if (!LastRight && ControlData.ButtonRight)
        {
            digitalG.set(!digitalG.value());
        }
        if (!LastY && ControlData.ButtonY)
        {
            digitalH.set(!digitalH.value());
        }
        if (!LastUp && ControlData.ButtonUp)
        {
            digitalF.set(!digitalF.value());
        }

        // 变量维护
        LastRight = ControlData.ButtonRight;
        LastY = ControlData.ButtonY;
        LastUp = ControlData.ButtonUp;

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
