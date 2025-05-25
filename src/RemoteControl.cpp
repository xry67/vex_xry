#include "vex.h"
#include "robot-config.h"
#include "RemoteControl.h"
#include "ChassisControl.h"
#include "units.h"

// 左电机控制
void LeftMotorControl(int val){
    LeftMotor1.spin(vex::forward, 0.128*val, voltageUnits::volt);
    LeftMotor2.spin(vex::forward, 0.128*val, voltageUnits::volt);
    LeftMotor3.spin(vex::forward, 0.128*val , voltageUnits::volt);

}

// 右电机控制
void RightMotorControl(int val){
    RightMotor1.spin(vex::forward, 0.128*val , voltageUnits::volt);
    RightMotor2.spin(vex::forward, 0.128*val , voltageUnits::volt);
    RightMotor3.spin(vex::forward, 0.128*val , voltageUnits::volt);
}

// 电机停止
void BaseMotorStop(int mode = 1) {
    if (mode == 2) {
        LeftMotor1.stop(hold);
        RightMotor1.stop(hold);    
        RightMotor3.stop(hold);
        LeftMotor2.stop(hold);
        LeftMotor3.stop(hold);
        RightMotor2.stop(hold);
    }
    else if (mode == 1) {
        LeftMotor1.stop(brake);
        RightMotor1.stop(brake);
        LeftMotor2.stop(brake);
        LeftMotor3.stop(brake);
        RightMotor2.stop(brake);
        RightMotor3.stop(brake);
    }
    else {
        LeftMotor1.stop(coast);
        RightMotor1.stop(coast);
        LeftMotor2.stop(coast);
        RightMotor2.stop(coast);
        LeftMotor3.stop(coast);
        RightMotor3.stop(coast);
    }
  
}

// 底盘控制
void ChassisControlFun(){

    int axis1 = Controller.Axis1.position();
    int axis2 = Controller.Axis2.position();
    int axis3 = Controller.Axis3.position();
    int axis4 = Controller.Axis4.position();


    float Sensitivity_1 = Sensitivity_Straight;
    float Sensitivity_2 = Sensitivity_Turn;

    // 精准操控模式
    if (Controller.ButtonR2.pressing()){
        Sensitivity_2 = Sensitivity_Straight * Sensitivity_Precision;
    }

    if (Controller.ButtonL2.pressing()){
        Sensitivity_1 = Sensitivity_Turn     * Sensitivity_Precision;
    }

    //  1：双 2：左 3：右 4：坦克
    if (RemoteControlMod == 2){
        if (abs(axis3) > blindCA || abs(axis4) > blindCA ){
            LeftMotorControl(axis3 * Sensitivity_1 + axis4 * Sensitivity_2);
            RightMotorControl(axis3 * Sensitivity_1 - axis4 * Sensitivity_2);
        }
        else{
            BaseMotorStop(0);
        }

    }

    if (RemoteControlMod == 3){
        if (abs(axis2) > blindCA || abs(axis1) > blindCA ){
            LeftMotorControl(axis2 * Sensitivity_1 + axis1 * Sensitivity_2);
            RightMotorControl(axis2 * Sensitivity_1 - axis1 * Sensitivity_2);
        }
        else{
            BaseMotorStop(0);
        }
    }

    if (RemoteControlMod == 1){
        // 死区判定
        if (abs(axis3) > blindCA || abs(axis1) > blindCA ){
            LeftMotorControl(axis3 * Sensitivity_1 + axis1 * Sensitivity_2);
            RightMotorControl(axis3 * Sensitivity_1 - axis1 * Sensitivity_2);
        }
        else{
            BaseMotorStop(0);
        }
    }

    if (RemoteControlMod == 4){
        if (abs(axis3) > blindCA || abs(axis2) > blindCA ){
            LeftMotorControl(axis3 * Sensitivity_1);
            RightMotorControl(axis2 * Sensitivity_1);
        }
        else{
            BaseMotorStop(0);
        }

    }
}

// 更新输入队列
void UpdataRemoteControlDataList(RemoteControlDataList *RemoteControlData_list){
    RemoteControlData InputData = RemoteControlData();  // 记录当时按钮输入

    InputData.axis1 = Controller.Axis1.position();
    InputData.axis2 = Controller.Axis2.position();
    InputData.axis3 = Controller.Axis3.position();
    InputData.axis4 = Controller.Axis4.position();

    InputData.ButtonA = Controller.ButtonA.pressing();
    InputData.ButtonB = Controller.ButtonB.pressing();
    InputData.ButtonX = Controller.ButtonX.pressing();
    InputData.ButtonY = Controller.ButtonY.pressing();

    InputData.ButtonDown  = Controller.ButtonDown.pressing();
    InputData.ButtonUp    = Controller.ButtonUp.pressing();
    InputData.ButtonLeft  = Controller.ButtonLeft.pressing();
    InputData.ButtonRight = Controller.ButtonRight.pressing();

    InputData.ButtonL1 = Controller.ButtonL1.pressing();
    InputData.ButtonL2 = Controller.ButtonL2.pressing();
    InputData.ButtonR1 = Controller.ButtonR1.pressing();
    InputData.ButtonR2 = Controller.ButtonR2.pressing();

    RemoteControlData_list->push(InputData);
} 

// 长按B检测
bool HoldButtonB(RemoteControlDataList *RemoteControlData_list, int limit = 100){
    int count = 0;
    for (int i=0; i<RemoteControlData_list->size(); i++){
        // 个数检测
        if (count >= limit){
            return true;
        }
        if (RemoteControlData_list->get(i).ButtonB == 0){
            // 屏幕更新
            PrintInLine(3);
            Controller.Screen.print("             退出(B: %2d)", count);

            // 震动
            if (count == limit * 0.9){
                std::cout << "---" << std::endl;
                Controller.rumble("..");
            }
            return false;
        }
        count += 1;
    }
    return true;
}

// 整体遥控
void RemoteControl(){

    RemoteControlDataList RemoteControlData_list(101);

    PrintInLine(1);
    Controller.Screen.print("遥控模式");

    PrintInLine(2);
    while (true){    
        // 输入更新
        UpdataRemoteControlDataList(&RemoteControlData_list);

        // 底盘控制
        ChassisControlFun();

        // 退出
        if (HoldButtonB(&RemoteControlData_list, 5)){
            return ;
        }

        wait(10, msec);
    }

}