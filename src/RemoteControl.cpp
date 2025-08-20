#include "vex.h"
#include "robot-config.h"
#include "RemoteControl.h"
#include "ChassisControl.h"
#include "units.h"
#include <sstream>

// AiTarget ai_target =  AiTarget();

// 长按B检测
bool HoldButtonB(int count, int limit = 100){
    // 个数检测
    if (count >= limit){
        return true;
    }

    PrintInLine(3);
    // Controller.Screen.print("             退出(B: %2d)", count);

    // 震动
    if (count == limit * 0.9){
        std::cout << "---" << std::endl;
        Controller.rumble("..");
    }
    return false;

}

// 整体遥控
void RemoteControlUnit(bool SaveFile, bool ReadFile, int OutTime, std::string file_name, double timeout){

    PrintInLine(1);
    if (stance == 'r'){
        PrintInLine(1);
        Controller.Screen.print("遥控模式-Red");
    }else{
        PrintInLine(1);
        Controller.Screen.print("遥控模式-Blue");
    }

    std::ifstream inFile;
    std::ofstream outFile;

    if (ReadFile){
        inFile.open(file_name);
    }

    if (SaveFile){
        outFile.open(file_name);
    }
   
    // ai_target.SetColor(Red);

    PrintInLine(2);

    // 吐球反转计数器
    int HaveBall = 0; 
    // 分球暂停计数器
    int ErrorBall = 0;

    // 大炮状态
    bool CannonMod = 0;
    bool LastUp    = 0;

    // 退出计数
    int HoldButtonBCount = 0;

    double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); 
    double now_time = start_time; 
    double out_time = 0;
    double last_time = start_time; 

    while (now_time - start_time < timeout){    
        last_time = now_time;
        now_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); 

        // 输入更新
        if (ReadFile){
            if (! ControlData.load(inFile)){
                std::cout << "end" << std::endl;
                break;
            }
            ControlData.print();
        }else{
            ControlData.input(Controller, now_time - last_time);
        }

        // 如果需要保存
        if (SaveFile){
            ControlData.save(outFile);
        }

        // 底盘控制
        ChassisRomoteControlSTS.Work(); 

        
        if (ControlData.axis2 > 95){
            stance = 'r';
            PrintInLine(1);
            Controller.Screen.print("遥控模式-Red  ");
            Controller.rumble(".");
        }
        if (ControlData.axis2 < -95){
            stance = 'b';
            PrintInLine(1);
            Controller.Screen.print("遥控模式-Blue");
            Controller.rumble("-");
        }

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
        // A 分到后背

        // 1 号
        if (ControlData.ButtonL1 || ControlData.ButtonL2 || ControlData.ButtonR2 || ControlData.ButtonR1){
            if (ControlData.ButtonL2){
                ToolMotor1.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                digitalH.set(1);
            }
            if (ControlData.ButtonL1){
                ToolMotor1.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                digitalH.set(1);
            }

            if (ControlData.ButtonR1){
                ToolMotor1.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
            }
            if (ControlData.ButtonR2){
                ToolMotor1.spin(forward, TrackSpeed, voltageUnits::volt);
            }

        }
        else{
            ToolMotor1.spin(forward, 0, voltageUnits::volt);
        }

        // 2号
        if (ControlData.ButtonL1 || ControlData.ButtonL2 || ControlData.ButtonR1 || ControlData.ButtonA || ControlData.ButtonX || ControlData.ButtonR2){
            if (ControlData.ButtonL2){
                ToolMotor2.spin(forward, TrackSpeed, voltageUnits::volt);
            }
            if (ControlData.ButtonL1){
                ToolMotor2.spin(forward, TrackSpeed, voltageUnits::volt);
            }
            if (ControlData.ButtonR1){
                ToolMotor2.spin(forward, 2, voltageUnits::volt);
            }
            // if (ControlData.ButtonLeft){
            //     ToolMotor2.spin(forward, TrackSpeed, voltageUnits::volt);
            // }
            // if (ControlData.ButtonUp){
            //     ToolMotor2.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
            // }

            if (ControlData.ButtonR2){
                ToolMotor2.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
            }

        }
        else{
            ToolMotor2.spin(forward, 0, voltageUnits::volt);
        }

        // 3号
        if (ControlData.ButtonL1 || ControlData.ButtonL2){
            if (ControlData.ButtonL2){
                ToolMotor3.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
            }
            if (ControlData.ButtonL1){
                ToolMotor3.spin(forward, TrackSpeed, voltageUnits::volt);
            }
        }
        else{
            ToolMotor3.spin(forward, 0, voltageUnits::volt);
        }

        
        if (ControlData.ButtonRight){
            digitalE.set(0);
        }

        if (ControlData.ButtonLeft){
            digitalE.set(1);
        }

        if (ControlData.ButtonDown){
            digitalH.set(0);
        }

        if (ControlData.ButtonUp && ! LastUp){
            CannonMod ^= 1;
            digitalF.set(CannonMod);
        }
        LastUp = ControlData.ButtonUp;

        if (ControlData.ButtonY){
          digitalG.set(0);
        }

        if (ControlData.ButtonB){
          digitalG.set(1);
        }

        // 退出
        if (auto_index == -1 && ControlData.ButtonB){
            HoldButtonBCount += 1;
            if (HoldButtonB(HoldButtonBCount, OutTime)){
                break;
            }
        }
        else{
            HoldButtonBCount = 0;
        }

        ErrorBall = std::max(ErrorBall - 1, 0);    // 反转计数器减小
        HaveBall = std::max(HaveBall - 1, 0);  // 暂停计数器减小

        out_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

        if (ReadFile){
            wait(std::max(0.0, ControlData.TimeWait -( out_time - now_time)), seconds);
        }
        else{
            wait(50, msec);
        }


    }

    // 如果需要保存
    if (SaveFile){
        outFile.close();
    }
    ChassisControlSTS.ChassisStoppingCoast();

}

// 整体遥控(手动)
void RemoteControl(){
    RemoteControlUnit(false, false, 50, "", 999999999999);
}

// 整体遥控（自动）
void AutoRemoteControl(std::string file_name, int timeout){
    CHMRCP.last_speed = 0;
    RemoteControlUnit(false, true, 2, file_name, timeout);
}