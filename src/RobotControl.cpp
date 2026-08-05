#include <vector>
#include <cstdlib>
#include "vex.h"
#include "robot-config.h"
#include "RobotControl.h"

// 指定电机旋转到指定位置
void MotorSpinFor(motor Motor, double v, double val, bool task)
{
    // Motor: 电机名称
    // v    : 速度
    // val  : 旋转位置
    // task : 多线程
    Motor.spinFor(forward, v, degrees, val, dps, !task);
}

// 移动到卡死为止
double moveToConNotMove(vex::motor motor_, double speed, int max_count)
{
    int count = 0;
    while (true)
    {
        motor_.spin(forward, speed, volt);
        // std::cout << "-->" << reversal_motor.velocity(dps) << std::endl;
        if (abs(motor_.velocity(dps)) <= 2)
        {
            count += 1;
        }
        else
        {
            count = 0;
        }
        if (count >= max_count)
        {
            break;
        }
        wait(10, msec);
    }
    motor_.spin(forward, 0, volt);
    return motor_.position(deg);
}

// 移动到卡死为止
double moveToConNotMove(vex::motor_group motor_, double speed, int max_count)
{
    int count = 0;
    while (true)
    {
        motor_.spin(forward, speed, volt);
        // std::cout << "-->" << reversal_motor.velocity(dps) << std::endl;
        if (abs(motor_.velocity(dps)) <= 2)
        {
            count += 1;
        }
        else
        {
            count = 0;
        }
        if (count >= max_count)
        {
            break;
        }
        wait(10, msec);
    }
    motor_.spin(forward, 0, volt);
    return motor_.position(deg);
}

// 将所有装置恢复到默认状态（马达停转）
void RestoreInitial()
{
    LeftMotor1.spin(vex::forward, 0, voltageUnits::volt);
    LeftMotor2.spin(vex::forward, 0, voltageUnits::volt);
    LeftMotor3.spin(vex::forward, 0, voltageUnits::volt);
    RightMotor1.spin(vex::forward, 0, voltageUnits::volt);
    RightMotor2.spin(vex::forward, 0, voltageUnits::volt);
    RightMotor3.spin(vex::forward, 0, voltageUnits::volt);

    ToolMotor1.spin(vex::forward, 0, voltageUnits::volt);
    ToolMotor2.spin(vex::forward, 0, voltageUnits::volt);
    ToolMotor3.spin(vex::forward, 0, voltageUnits::volt);
    ToolMotor4.spin(vex::forward, 0, voltageUnits::volt);
    ToolMotor5.spin(vex::forward, 0, voltageUnits::volt);
    ToolMotor6.spin(vex::forward, 0, voltageUnits::volt);
    // ToolMotor4.spin(vex::forward, 0, voltageUnits::volt);
}

// 等待陀螺仪校准完成
void WaitInertialCalibrating()
{
    if (InertialIsReallyGood)
    {
        return;
    }

    Controller.Screen.clearScreen(); // 清除屏幕

    WaitStr wait_str = WaitStr();

    while (!InertialIsReallyGood)
    {

        PrintInLine(2);
        char formattedString[50]; // 创建字符数组来存储格式化字符串
        // 使用 snprintf 生成格式化字符串
        snprintf(formattedString, 50, "-> 初始化中 %s <-", wait_str.GetNextStr());
        // 使字符串居中
        CenterStr(formattedString);

        Controller.Screen.print(formattedString);

        wait(100, msec);
    }
}

// 保证陀螺仪矫正完成
int InertialCalibrating()
{
    if (InertialIsReallyGood)
    {
        return 0;
    }

    WaitStr wait_str = WaitStr();

    while (ChassisInertial.isCalibrating())
    {
        wait(10, msec);
    }

    // 校准失败则重新校准
    if (CheckInertialCalibrationFailed(1000, 0.1 + 0.2 * (InertialCalibrationCount % 5)))
    {
        Controller.rumble(".");
        ChassisInertial.calibrate(); // 陀螺仪校准
        InertialCalibrationCount += 1;
        InertialCalibrating();
    }

    InertialIsReallyGood = true;
    return 0;
}

// 检测是否飘值
bool CheckInertialCalibrationFailed(double time, double max_error)
{
    // std::cout << "in" << std::endl;
    double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); // 运动开始时间
    double now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();   // 运动开始时间
    double now_angle = ChassisInertial.rotation();
    double last_angle = now_angle;
    double total_error = 0;

    WaitStr wait_str = WaitStr();

    while (now_time - start_time < time / 1000)
    {
        now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); // 运动开始时间
        now_angle = ChassisInertial.rotation();

        total_error += std::abs(last_angle - now_angle);
        last_angle = now_angle;
        // std::cout << total_error << std::endl;
        wait(10, msec);
    }

    return total_error > max_error;
}