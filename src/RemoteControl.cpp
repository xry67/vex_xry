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

bool halfauto = false; // 半自动模式
bool RemoteControlChildFlag = true;  // 掌管子线程生死的变量
int RemoteControl_child()
{
    bool LastB = false;
    bool LastRight = false;
    bool LastUp = false;
    bool LastR1 = false;
    bool LastR2 = false;
    bool LastX = false;
    bool LastA = false;
    bool LastDown = false;
    while (RemoteControlChildFlag)
    {

        // 半自动 预备
        if (!ControlData.ButtonDown && LastDown)
        {
            halfauto = true;
            // 判断夹子的位置
            std::cout << Claw.claw_reversal << " " << Claw.reversal_move_flag << std::endl;
            if (Claw.reversal_digital.value() != 0)
            {
                Claw.reversal_digital.set(1);
            }
            if (Claw.claw_mod == 1 && Claw.mod_move_flag == false)
            {
                Claw.changeMod(0);
            }

            Claw.LiftMove(1000);
            halfauto = false;
        }
        // 半自动 夹！
        if (!ControlData.ButtonRight && LastRight)
        {
            halfauto = true;
            ToolMotor3.spin(forward, 0, volt);
            // 先执行一个这个动作
            if (Claw.reversal_digital.value() != 0)
            {
                Claw.reversal_digital.set(0);
            }
            if (Claw.claw_mod == 1 && Claw.mod_move_flag == false)
            {
                Claw.changeMod(0);
            }

            Claw.LiftMove(1000);

            while (Claw.mod_move_flag)
            {
                wait(10, msec);
            }
            Claw.moveing_flag = true;
            ToolMotor6.spin(forward, -13, volt);

            // 移动到限位触发
            for (int i = 0; i < 150; i++)
            {
                if (!liftLimit.pressing())
                {
                    Claw.lift_motor1.spin(forward, -12, volt);
                    Claw.lift_motor2.spin(forward, -12, volt);
                }
                else
                {
                    break;
                }
                wait(10, msec);
            }

            Claw.lift_motor1.spin(forward, 0, volt);
            Claw.lift_motor2.spin(forward, 0, volt);

            wait(0.5, seconds);

            Claw.LiftMove(200);

            // 两个一起动
            Claw.lift_move_flag = true;
            Claw.mod_move_flag = true;
            while (Claw.lift_motor1.position(rotationUnits::deg) < 400)
            {
                Claw.lift_motor1.spin(forward, 7, volt);
                Claw.lift_motor2.spin(forward, 7, volt);
                Claw.mod_motor.spin(forward, 6, volt);
                std::cout << "->" << Claw.lift_move_flag << Claw.lift_motor1.position(rotationUnits::deg) << std::endl;
                wait(10, msec);
            }
            Claw.mod_move_flag = false;
            Claw.lift_move_flag = false;
            Claw.lift_motor1.spin(forward, 0, volt);
            Claw.lift_motor2.spin(forward, 0, volt);

            // Claw.LiftMove(1000);
            Claw.changeMod();

            wait(300, msec);
            halfauto = false;
        }
        LastR1 = ControlData.ButtonR1;
        LastR2 = ControlData.ButtonR2;
        LastUp = ControlData.ButtonUp;
        LastX = ControlData.ButtonX;
        LastA = ControlData.ButtonA;
        LastRight = ControlData.ButtonRight;
        LastB = ControlData.ButtonB;
        LastDown = ControlData.ButtonDown;
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

    // 分球暂停计数器
    int ErrorBall1 = 0;
    int ErrorBall2 = 0;

    // 退出计数
    int HoldButtonBCount = 0;

    // 卡死检测
    int MotorBlocked = true;
    int MotorBlocked_count = 0;

    int intake_mod = 0; // intake状态

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
    bool LastUp = false;

    RemoteControlChildFlag = true;
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
        // ChangeStance();

        // 夹子收吐
        if (ControlData.ButtonR1)
        {
            if (Claw.claw_mod == 1)
            {
                ToolMotor6.spin(forward, -13, volt);
            }
            ToolMotor3.spin(forward, -13, volt);
        }
        else if (ControlData.ButtonR2)
        {
            ToolMotor6.spin(forward, 3, volt);
            ToolMotor3.spin(forward, 13, volt);
        }
        else
        {
            if (!halfauto)
            {
                ToolMotor6.spin(forward, 0, volt);
            }
            ToolMotor3.spin(forward, 0, volt);
        }

        // 爪子 折叠
        if (LastUp && !ControlData.ButtonUp)
        {
            Claw.changeMod();
        }

        // 夹子 反转
        if (LastB && !ControlData.ButtonB)
        {
            Claw.reversal();
        }

        // 钩子
        if (LastX && !ControlData.ButtonX)
        {
            digitalH.set(!digitalH.value());
        }

        // 抬升
        // Brain.Screen.printAt(100, 120, "%f", ToolMotor1.position(deg));
        if (std::abs(ControlData.axis2) > 10)
        {
            // ToolMotor1.spin(forward, -ControlData.axis2, pct);
            // ToolMotor2.spin(forward, -ControlData.axis2, pct);
        }
        else if (ControlData.ButtonL1)
        {
            ToolMotor1.spin(forward, 13, volt);
            ToolMotor2.spin(forward, 13, volt);
        }
        else if (ControlData.ButtonL2 && !liftLimit.pressing())
        {
            ToolMotor1.spin(forward, -13, volt);
            ToolMotor2.spin(forward, -13, volt);
        }
        else if (!halfauto)
        {
            // ToolMotor1.setStopping(hold);
            // ToolMotor2.setStopping(hold);
            ToolMotor1.stop();
            ToolMotor2.stop();
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
        LastUp = ControlData.ButtonUp;
    }

    RemoteControlChildFlag = false; // 关闭子线程
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
