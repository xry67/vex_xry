#include <string>
#include <vector>
#include <memory>
#include "robot-config.h"
#include "units.h"
#include "ChassisRemoteControl.h"

RemoteControlData ControlData = RemoteControlData(); // 记录遥控数据

// 左电机控制
void LeftMotorControl(int val)
{
    // Brain.Screen.printAt( 100, 50, "%f" , 0.128*val);
    LeftMotor1.spin(vex::forward, 0.128 * val, voltageUnits::volt);
    LeftMotor2.spin(vex::forward, 0.128 * val, voltageUnits::volt);
    LeftMotor3.spin(vex::forward, 0.128 * val, voltageUnits::volt);
}

// 右电机控制
void RightMotorControl(int val)
{
    RightMotor1.spin(vex::forward, 0.128 * val, voltageUnits::volt);
    RightMotor2.spin(vex::forward, 0.128 * val, voltageUnits::volt);
    RightMotor3.spin(vex::forward, 0.128 * val, voltageUnits::volt);
}

// 电机停止
void BaseMotorStop(int mode = 1)
{
    if (mode == 2)
    {
        LeftMotor1.stop(hold);
        RightMotor1.stop(hold);
        RightMotor3.stop(hold);
        LeftMotor2.stop(hold);
        LeftMotor3.stop(hold);
        RightMotor2.stop(hold);
    }
    else if (mode == 1)
    {
        LeftMotor1.stop(brake);
        RightMotor1.stop(brake);
        LeftMotor2.stop(brake);
        LeftMotor3.stop(brake);
        RightMotor2.stop(brake);
        RightMotor3.stop(brake);
    }
    else
    {
        LeftMotor1.stop(coast);
        RightMotor1.stop(coast);
        LeftMotor2.stop(coast);
        RightMotor2.stop(coast);
        LeftMotor3.stop(coast);
        RightMotor3.stop(coast);
    }
}

// 双摇杆
void ChassisBothHandsRomoteControl()
{
    int axis1 = ControlData.axis1;
    int axis2 = ControlData.axis2;
    int axis3 = ControlData.axis3;
    int axis4 = ControlData.axis4;
    // 死区判定
    if (std::abs(axis3) > blindCA || std::abs(axis1) > blindCA)
    {
        LeftMotorControl(axis3 * Sensitivity_Straight + axis1 * Sensitivity_Turn);
        RightMotorControl(axis3 * Sensitivity_Straight - axis1 * Sensitivity_Turn);
    }
    else
    {
        BaseMotorStop(0);
    }
}

// 双摇杆-反转
void ChassisBothHandsRomoteControl_reverse()
{
    int axis1 = ControlData.axis1;
    int axis2 = ControlData.axis2;
    int axis3 = ControlData.axis3;
    int axis4 = ControlData.axis4;
    // 死区判定
    if (std::abs(axis3) > blindCA || std::abs(axis1) > blindCA)
    {
        LeftMotorControl(-(axis3 * Sensitivity_Straight - axis1 * Sensitivity_Turn));
        RightMotorControl(-(axis3 * Sensitivity_Straight + axis1 * Sensitivity_Turn));
    }
    else
    {
        BaseMotorStop(0);
    }
}

// 左单
void ChassisLeftHandsRomoteControl()
{
    int axis1 = ControlData.axis1;
    int axis2 = ControlData.axis2;
    int axis3 = ControlData.axis3;
    int axis4 = ControlData.axis4;
    if (std::abs(axis3) > blindCA || std::abs(axis4) > blindCA)
    {
        LeftMotorControl(axis3 * Sensitivity_Straight + axis4 * Sensitivity_Turn);
        RightMotorControl(axis3 * Sensitivity_Straight - axis4 * Sensitivity_Turn);
    }
    else if (std::abs(axis1) > blindCA || std::abs(axis2) > blindCA)
    {
        double current_position = ChassisInertial.rotation(); // 当前位置

        double current_position_norm = NormsAngle(current_position);
        double taget_angle_position = calculateAngle(axis1, axis2);

        double angle_difference = angleDifference(current_position_norm, taget_angle_position);
        LeftMotorControl(angle_difference);
        RightMotorControl(angle_difference * -1);
    }
    else
    {
        BaseMotorStop(0);
    }
}

// 右单
void ChassisRightHandsRomoteControl()
{
    int axis1 = ControlData.axis1;
    int axis2 = ControlData.axis2;
    int axis3 = ControlData.axis3;
    int axis4 = ControlData.axis4;
    if (std::abs(axis2) > blindCA || std::abs(axis1) > blindCA)
    {
        LeftMotorControl(axis2 * Sensitivity_Straight + axis1 * Sensitivity_Turn);
        RightMotorControl(axis2 * Sensitivity_Straight - axis1 * Sensitivity_Turn);
    }
    else
    {
        BaseMotorStop(0);
    }
}

// 坦克
void ChassisTankRomoteControl()
{
    int axis1 = ControlData.axis1;
    int axis2 = ControlData.axis2;
    int axis3 = ControlData.axis3;
    int axis4 = ControlData.axis4;
    if (std::abs(axis3) > blindCA || std::abs(axis2) > blindCA)
    {
        LeftMotorControl(axis3 * Sensitivity_Straight);
        RightMotorControl(axis2 * Sensitivity_Straight);
    }
    else
    {
        BaseMotorStop(0);
    }
}

ChassisHumanMachineRomoteControlPara CHMRCP = ChassisHumanMachineRomoteControlPara();

// 人机遥控
void ChassisHumanMachineRomoteControl()
{
    int axis1 = ControlData.axis1;
    int axis2 = ControlData.axis2;
    int axis3 = ControlData.axis3;
    int axis4 = ControlData.axis4;
    if (std::abs(axis3) > blindCA)
    {
        LeftMotorControl(CHMRCP.GetSpeed(axis3 * Sensitivity_Straight));
        RightMotorControl(CHMRCP.GetSpeed(axis3 * Sensitivity_Straight));
    }
    else if (std::abs(axis1) > blindCA || std::abs(axis2) > blindCA)
    {
        double current_position = ChassisInertial.rotation(); // 当前位置

        double current_position_norm = NormsAngle(current_position);
        double taget_angle_position = calculateAngle(axis1, axis2);

        double angle_difference = angleDifference(current_position_norm, taget_angle_position);
        LeftMotorControl(angle_difference);
        RightMotorControl(angle_difference * -1);
    }
    else
    {
        BaseMotorStop(0);
    }
}

// 双摇杆/ 转向特调
void ChassisBothHandsRomoteControl_Turn()
{
    int axis1 = ControlData.axis1;
    int axis3 = ControlData.axis3;

    // double temp_Sensitivity_Straight = Sensitivity_Straight * std::abs(axis1 / 100.0);
    double temp_Sensitivity_Turn = Sensitivity_Turn + std::abs((double)axis3 / 100.0) * 0.5;

    // 死区判定
    if (std::abs(axis3) > blindCA || std::abs(axis1) > blindCA)
    {
        LeftMotorControl(axis3 * Sensitivity_Straight + axis1 * temp_Sensitivity_Turn);
        RightMotorControl(axis3 * Sensitivity_Straight - axis1 * temp_Sensitivity_Turn);
    }
    else
    {
        BaseMotorStop(0);
    }
}

// 无底盘
void ChassisBothHandsRomoteControl_NAN()
{
}

void AddRomoteFun()
{
    ChassisRomoteControlList.push_back(ChassisRomoteControlParams("双摇杆", ChassisBothHandsRomoteControl)); // 双摇杆加入

    ChassisRomoteControlList.push_back(ChassisRomoteControlParams("超级形态", ChassisBothHandsRomoteControl_reverse)); // 超级形态
    ChassisRomoteControlList.push_back(ChassisRomoteControlParams("左单摇杆", ChassisLeftHandsRomoteControl));         // 左单摇杆
    ChassisRomoteControlList.push_back(ChassisRomoteControlParams("右单摇杆", ChassisRightHandsRomoteControl));        // 右单摇杆
    ChassisRomoteControlList.push_back(ChassisRomoteControlParams("坦克遥控", ChassisTankRomoteControl));              // 坦克遥控
    ChassisRomoteControlList.push_back(ChassisRomoteControlParams("人机遥控", ChassisHumanMachineRomoteControl));      // 坦克遥控
    ChassisRomoteControlList.push_back(ChassisRomoteControlParams("双-转向", ChassisBothHandsRomoteControl_Turn));     // 双-转向
    ChassisRomoteControlList.push_back(ChassisRomoteControlParams("无底盘", ChassisBothHandsRomoteControl_NAN));       // 双-转向
}