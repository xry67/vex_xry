#include "vex.h"
#include "robot-config.h"
#include "RemoteControl.h"
#include "units.h"

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

        char* formattedString = "->左摇杆控制<-";
        CenterStr(formattedString);
        Controller.Screen.print(formattedString);
        LeftMotorGroup.spin(forward, axis3 * Sensitivity_1 + axis4 * Sensitivity_2, voltageUnits::volt);
        RightMotorGroup.spin(forward, axis3 * Sensitivity_1 - axis4 * Sensitivity_2, voltageUnits::volt);
    }

    if (RemoteControlMod == 2){
        float Sensitivity_1 = 1;
        float Sensitivity_2 = 1;

        char* formattedString = "->右摇杆控制<-";
        CenterStr(formattedString);
        Controller.Screen.print(formattedString);
        LeftMotorGroup.spin(forward, axis2 * Sensitivity_1 + axis1 * Sensitivity_2, voltageUnits::volt);
        RightMotorGroup.spin(forward, axis2 * Sensitivity_1 - axis1 * Sensitivity_2, voltageUnits::volt);
    }

    if (RemoteControlMod == 3){
        float Sensitivity_1 = 1;
        float Sensitivity_2 = 1;

        char* formattedString = "->双摇杆控制<-";
        CenterStr(formattedString);
        Controller.Screen.print(formattedString);
        LeftMotorGroup.spin(forward, axis3 * Sensitivity_1 + axis1 * Sensitivity_2, voltageUnits::volt);
        RightMotorGroup.spin(forward, axis3 * Sensitivity_1 - axis1 * Sensitivity_2, voltageUnits::volt);
    }

    if (RemoteControlMod == 4){
        float Sensitivity_1 = 1;
        // float Sensitivity_2 = 1;
        
        char* formattedString = "->坦克摇杆控制<-";
        CenterStr(formattedString);
        Controller.Screen.print(formattedString);
        LeftMotorGroup.spin(forward, axis3 * Sensitivity_1, voltageUnits::volt);
        RightMotorGroup.spin(forward, axis2 * Sensitivity_1, voltageUnits::volt);
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
        if (count >= limit){
            return true;
        }
        if (RemoteControlData_list->get(i).ButtonB == 0){
            // 屏幕更新
            PrintInLine(3);
            Controller.Screen.print("             退出(B: %2d)", count);
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
        ChassisControl();

        // 其他控制

        // 退出
        if (HoldButtonB(&RemoteControlData_list, 50)){
            return ;
        }

        wait(10, msec);
    }

}