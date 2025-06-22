#include "vex.h"
#include "robot-config.h"
#include "RemoteControl.h"
#include "ChassisControl.h"
#include "units.h"

AiTarget ai_target =  AiTarget();
// 移动气动检测
int MoveCount = 0;

// 左电机控制
void LeftMotorControl(int val){
    // Brain.Screen.printAt( 100, 50, "%f" , 0.128*val);
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
    float auto_target_yaw = 0.0;  // 智能对齐偏转

    // 精准操控模式
    // if (Controller.ButtonDown.pressing()){
    //     Sensitivity_2 = Sensitivity_Straight * Sensitivity_Precision;
    //     Sensitivity_1 = Sensitivity_Turn     * Sensitivity_Precision;
    // }

    int Move_Threshold = 30;

    //  1：双 2：左 3：右 4：坦克
    if (RemoteControlMod == 2){
        if (abs(axis3) > blindCA || abs(axis4) > blindCA ){
            LeftMotorControl(axis3 * Sensitivity_1 + axis4 * Sensitivity_2 + auto_target_yaw);
            RightMotorControl(axis3 * Sensitivity_1 - axis4 * Sensitivity_2 - auto_target_yaw);

            if (abs(axis3) >= Move_Threshold || abs(axis4) >= Move_Threshold){
                MoveCount += 1;
            }else{
                MoveCount -= 1;
            }
        }
        else{
            BaseMotorStop(0);
            MoveCount = 0;
        }

    }

    else if (RemoteControlMod == 3){
        if (abs(axis2) > blindCA || abs(axis1) > blindCA ){
            LeftMotorControl(axis2 * Sensitivity_1 + axis1 * Sensitivity_2 + auto_target_yaw);
            RightMotorControl(axis2 * Sensitivity_1 - axis1 * Sensitivity_2 - auto_target_yaw);

            if (abs(axis2) >= Move_Threshold || abs(axis1) >= Move_Threshold){
                MoveCount += 1;
            }else{
                MoveCount -= 1;
            }
        }
        else{
            BaseMotorStop(0);
            MoveCount = 0;
        }
    }

    else if (RemoteControlMod == 1){
        // 死区判定
        if (abs(axis3) > blindCA || abs(axis1) > blindCA ){
            LeftMotorControl(axis3 * Sensitivity_1 + axis1 * Sensitivity_2 + auto_target_yaw);
            RightMotorControl(axis3 * Sensitivity_1 - axis1 * Sensitivity_2 - auto_target_yaw);

            if (abs(axis3) >= Move_Threshold || abs(axis1) >= Move_Threshold){
                MoveCount += 1;
            }else{
                MoveCount -= 1;
            }
        }
        else{
            BaseMotorStop(0);
            MoveCount = 0;
        }
    }

    else if (RemoteControlMod == 4){
        if (abs(axis3) > blindCA || abs(axis2) > blindCA ){
            LeftMotorControl(axis3 * Sensitivity_1 + auto_target_yaw);
            RightMotorControl(axis2 * Sensitivity_1 - auto_target_yaw);

            if (abs(axis3) >= Move_Threshold || abs(axis2) >= Move_Threshold){
                MoveCount += 1;
            }else{
                MoveCount -= 1;
            }
        }
        else{
            BaseMotorStop(0);
            MoveCount = 0;
        }

    }

    if (MoveCount > 100){
        digitalH.set(1);
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
bool HoldButtonB(int count, int limit = 100){
    // 个数检测
    if (count >= limit){
        return true;
    }

    PrintInLine(3);
    Controller.Screen.print("             退出(B: %2d)", count);

    // 震动
    if (count == limit * 0.9){
        std::cout << "---" << std::endl;
        Controller.rumble("..");
    }
    return false;

}

// 整体遥控
void RemoteControl(){

    RemoteControlDataList RemoteControlData_list(101);

    PrintInLine(1);
    if (stance == 'r'){
        Controller.Screen.print("遥控模式-Red");
    }else{
        Controller.Screen.print("遥控模式-Blue");
    }
   

    ai_target.SetColor(Red);

    PrintInLine(2);

    // 吐球反转计数器
    int HaveBall = 0; 
    // 分球暂停计数器
    int ErrorBall = 0;

    // 退出计数
    int HoldButtonBCount = 0;

    while (true){    
        // 输入更新
        // UpdataRemoteControlDataList(&RemoteControlData_list);

        // 底盘控制
        ChassisControlFun();

        // 红色
        if (OpticalHaveRed() || OpticalHaveBlue()){
            HaveBall = 50;
        }else{
            HaveBall = 0;
        }

        if ((stance == 'r' && OpticalHaveBlue()) || (stance == 'b' && OpticalHaveRed())){
            ErrorBall = 20;
        }
        else if ((stance == 'b' && OpticalHaveBlue()) || (stance == 'r' && OpticalHaveRed())){
            ErrorBall = 0;
        }

        // R1 R2 -  吸球 吐球
        // L1 高层 吐
        // L2 中层 吐
        if (Controller.ButtonL1.pressing() || Controller.ButtonL2.pressing()){
            if (Controller.ButtonL2.pressing()){
                ToolMotor2.spin(forward, TrackSpeed, voltageUnits::volt);
                ToolMotor3.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                digitalH.set(0);
            }
            if (Controller.ButtonL1.pressing()){
                ToolMotor2.spin(forward, TrackSpeed, voltageUnits::volt);
                ToolMotor3.spin(forward, TrackSpeed, voltageUnits::volt);
                digitalH.set(0);
            }
        }
        else{
            ToolMotor2.spin(forward, 0, voltageUnits::volt);
            ToolMotor3.spin(forward, 0, voltageUnits::volt);
        }

        if (Controller.ButtonR1.pressing() || Controller.ButtonR2.pressing()){
            if (Controller.ButtonR1.pressing()){
                ToolMotor1.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
            }
            if (Controller.ButtonR2.pressing()){
                ToolMotor1.spin(forward, TrackSpeed, voltageUnits::volt);
            }
        }
        else{
            ToolMotor1.spin(forward, 0, voltageUnits::volt);
        }
        

        if (Controller.ButtonY.pressing()){
            digitalG.set(1);
        }
        if (Controller.ButtonB.pressing()){
            digitalG.set(0);
        }
        if (Controller.ButtonDown.pressing()){
            digitalH.set(1);
        }

        // 底盘控制
        ChassisControlFun();

        // 退出
        if (auto_index == -1 && Controller.ButtonB.pressing()){
            HoldButtonBCount += 1;
            if (HoldButtonB(HoldButtonBCount, 50)){
                break;
            }
        }
        else{
            HoldButtonBCount = 0;
        }

        ErrorBall = std::max(ErrorBall - 1, 0);    // 反转计数器减小
        HaveBall = std::max(HaveBall - 1, 0);  // 暂停计数器减小

        wait(10, msec);
    }

}