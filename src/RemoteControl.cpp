#include "RemoteControl.h"
#include "vex.h"

// 底盘控制
void ChassisControl(){

    float axis1 = Controller.Axis1.position();
    float axis2 = Controller.Axis2.position();
    float axis3 = Controller.Axis3.position();
    float axis4 = Controller.Axis4.position();

    //  1：左 2：右 3：双 4：坦克
    if (RemoteControlMod == 1){
        float Sensitivity_1 = 1;
        float Sensitivity_2 = 1;

        LeftMotorGroup.spin(forward, axis3 * Sensitivity_1 + axis4 * Sensitivity_2, voltageUnits::volt);
        RightMotorGroup.spin(forward, axis3 * Sensitivity_1 - axis4 * Sensitivity_2, voltageUnits::volt);
    }

    if (RemoteControlMod == 2){
        float Sensitivity_1 = 1;
        float Sensitivity_2 = 1;

        LeftMotorGroup.spin(forward, axis2 * Sensitivity_1 + axis1 * Sensitivity_2, voltageUnits::volt);
        RightMotorGroup.spin(forward, axis2 * Sensitivity_1 - axis1 * Sensitivity_2, voltageUnits::volt);
    }

    if (RemoteControlMod == 3){
        float Sensitivity_1 = 1;
        float Sensitivity_2 = 1;

        LeftMotorGroup.spin(forward, axis3 * Sensitivity_1 + axis1 * Sensitivity_2, voltageUnits::volt);
        RightMotorGroup.spin(forward, axis3 * Sensitivity_1 - axis1 * Sensitivity_2, voltageUnits::volt);
    }

    if (RemoteControlMod == 3){
        float Sensitivity_1 = 1;
        // float Sensitivity_2 = 1;

        LeftMotorGroup.spin(forward, axis3 * Sensitivity_1, voltageUnits::volt);
        RightMotorGroup.spin(forward, axis2 * Sensitivity_1, voltageUnits::volt);
    }
}

// 整体遥控
void RemoteControl(){
    // 底盘控制
    ChassisControl();

    // 其他控制
}