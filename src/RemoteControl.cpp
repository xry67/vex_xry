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
    while (1)
    {

        // 半自动 预备
        if (!ControlData.ButtonDown && LastDown)
        {
            halfauto = true;
            // 判断夹子的位置
            std::cout << Claw.claw_reversal << " " << Claw.reversal_move_flag << std::endl;
            if (Claw.claw_reversal && Claw.reversal_move_flag == false)
            {
                std::cout << "reversal" << std::endl;
                Claw.reversal();
            }
            if (Claw.claw_mod != 1 && Claw.mod_move_flag == false)
            {
                Claw.changeMod(1);
            }

            Claw.LiftMove(StartLiftPosititon + 21); // 38
            halfauto = false;
        }
        // 半自动 夹！
        if (!ControlData.ButtonRight && LastRight)
        {
            halfauto = true;
            ToolMotor3.spin(forward, 0, volt);
            // 先执行一个这个动作
            if (Claw.claw_reversal && Claw.reversal_move_flag == false)
            {
                Claw.reversal();
            }
            if (Claw.claw_mod != 2 && Claw.mod_move_flag == false)
            {
                Claw.changeMod(2);
            }

            while (Claw.Rotation.angle() < StartLiftPosititon + 22)
            {
                Claw.lift_motor1.spin(forward, -7, volt);
                Claw.lift_motor2.spin(forward, -7, volt);
                wait(10, msec);
            }
            Claw.lift_motor1.stop();
            Claw.lift_motor2.stop();

            while (Claw.mod_move_flag || Claw.reversal_move_flag)
            {
                wait(10, msec);
            }

            Claw.moveing_flag = true;
            ToolMotor6.spin(forward, -8, volt);
            int StrtLiftTime = GetNowTime();
            Claw.LiftMove(StartLiftPosititon + 12);
            wait(0.3 - (GetNowTime() - StrtLiftTime), sec);
            ToolMotor6.spin(forward, -8, volt);

            // 两个一起动
            Claw.mod_move_flag = 1;
            while (Claw.Rotation.angle() < StartLiftPosititon + 18)
            {
                Claw.lift_motor1.spin(forward, -3.8, volt);
                Claw.lift_motor2.spin(forward, -3.8, volt);
                Claw.mod_motor.spin(forward, -13, volt);
                wait(10, msec);
            }
            Claw.mod_move_flag = 0;
            Claw.changeMod();
            Claw.LiftMove(StartLiftPosititon + 22);

            while (Claw.mod_move_flag)
            {
                wait(10, msec);
            }
            halfauto = false;
        }

        // 爪子 折叠
        if (LastUp && !ControlData.ButtonUp)
        {
            Claw.changeMod();
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
    int MotorBlocked5 = true;
    int MotorBlocked5_count = 0; // 马达5卡死检测
    bool Motor5_flag = false;

    double LiftUpSpeed = 0;   // 抬升速度
    double LiftDownSpeed = 0; // 抬升速度
    double LiftSpeed = 1;     // 抬升速度

    int intake_mod = 1; // intake状态

    // 时间变量
    double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    double now_time = start_time;
    double out_time = 0;
    double last_time = start_time;

    bool LastX = false;
    bool LastB = false;
    bool LastY = false;
    bool LastL1 = false;
    bool LastL2 = false;
    bool LastDown = false;
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

        // 底盘控制
        ChassisRomoteControlSTS.Work();
        // ChangeStance();

        if (ControlData.ButtonR1 || ControlData.ButtonR2)
        {
            if (std::abs(ToolMotor5.velocity(pct)) <= 3)
            {
                MotorBlocked5_count++;
            }

            if (MotorBlocked5_count > 10)
            {
                MotorBlocked5 = true;
            }
        }
        else
        {
            MotorBlocked5 = false;
            MotorBlocked5_count = 0;
        }

        // 夹子收吐
        if (ControlData.ButtonR1)
        {
            if (Claw.claw_mod == 0)
            {
                ToolMotor6.spin(forward, -13, volt);
            }

            ToolMotor3.spin(forward, -13, volt);

            // if (!MotorBlocked5)
            // {
            //     ToolMotor5.spin(forward, 13, volt);
            // }
            // else
            // {
            //     ToolMotor5.spin(forward, 2, volt);
            // }
            Motor5_flag = true;
        }
        else if (ControlData.ButtonR2)
        {
            ToolMotor3.spin(forward, 13, volt);

            if (!MotorBlocked5)
            {
                ToolMotor5.spin(forward, -13, volt);
            }
            else
            {
                ToolMotor5.spin(forward, 0, volt);
            }

            Motor5_flag = false;
        }
        else
        {
            if (!halfauto)
            {
                ToolMotor6.spin(forward, 0, volt);
            }
            ToolMotor3.spin(forward, 0, volt);

            if (Motor5_flag)
            {
                ToolMotor5.spin(forward, 2, volt);
            }
            else
            {
                ToolMotor5.spin(forward, 0, volt);
            }
            MotorBlocked5 = false;
            // MotorBlocked5_count = 0;
        }

        // 夹子 反转
        if (LastB && !ControlData.ButtonB)
        {
            Claw.reversal();
        }

        // 夹子 开合
        if (!LastX && ControlData.ButtonX)
        {
            digitalA.set(!digitalA.value());
        }

        // 抬升
        // Brain.Screen.printAt(100, 120, "%f", ToolMotor1.position(deg));
        if (ControlData.ButtonL1)
        {
            LiftUpSpeed -= LiftSpeed;
            LiftDownSpeed = 0;
            ToolMotor1.spin(forward, LiftUpSpeed, volt);
            ToolMotor2.spin(forward, LiftUpSpeed, volt);
        }
        else if (ControlData.ButtonL2)
        {
            LiftDownSpeed += LiftSpeed;
            LiftUpSpeed = 0;
            ToolMotor1.spin(forward, LiftDownSpeed, volt);
            ToolMotor2.spin(forward, LiftDownSpeed, volt);
        }
        else if (!halfauto)
        {
            LiftUpSpeed = 0;
            LiftDownSpeed = 0;
            ToolMotor1.setStopping(hold);
            ToolMotor2.setStopping(hold);
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

        LastY = ControlData.ButtonY;
        LastX = ControlData.ButtonX;
        LastB = ControlData.ButtonB;
        LastL1 = ControlData.ButtonL1;
        LastL2 = ControlData.ButtonL2;
        LastDown = ControlData.ButtonDown;
        LastUp = ControlData.ButtonUp;
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
