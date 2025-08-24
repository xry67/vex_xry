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
        Controller.rumble("..");
    }
    return false;

}

// 消分一代车遥控，双颜色传感器分球
void RemoteControl_1(bool SaveFile, bool ReadFile, int OutTime, std::string file_name, double timeout){
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
    int ErrorBall1 = 0;
    int ErrorBall2 = 0;

    // 大炮状态
    bool CannonMod = 0;
    bool LastUp    = 0;

    // 前后气动联动设置
    int digitalGOverCount = 0;
    int digitalDOverCount = 0;
    int digitalOverCount = 5;
    int digitalMod       = 1; // 1：前开后闭，0：前闭后开
    bool digital_move_finish = false;

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
        if (OpticalHaveRed(Optical1) || OpticalHaveBlue(Optical1)){
            HaveBall = 50;
        }else{
            HaveBall = 0;
        }

        if ((stance == 'r' && OpticalHaveBlue(Optical1)) || (stance == 'b' && OpticalHaveRed(Optical1))){
            ErrorBall1 = 20;
        }
        else if ((stance == 'b' && OpticalHaveBlue(Optical1)) || (stance == 'r' && OpticalHaveRed(Optical1))){
            ErrorBall1 = 0;
        }

        if ((stance == 'r' && OpticalHaveBlue(Optical2)) || (stance == 'b' && OpticalHaveRed(Optical2))){
            ErrorBall2 = 20;
        }
        else if ((stance == 'b' && OpticalHaveBlue(Optical2)) || (stance == 'r' && OpticalHaveRed(Optical2))){
            ErrorBall2 = 0;
        }
        else{
          ErrorBall2 = -1;
        }


        // R1 R2 -  吸球 吐球
        // L1 高层 吐
        // L2 中层 吐
        // A 分到后背

        // 1 号
        if (ControlData.ButtonL1 || ControlData.ButtonL2 || ControlData.ButtonR2 || ControlData.ButtonR1){
            if (ControlData.ButtonL2){
                ToolMotor1.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                digitalH.set(0);
                digitalE.set(1);
            }
            if (ControlData.ButtonL1){
                ToolMotor1.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                digitalH.set(0);
                digitalE.set(1);
            }

            if (ControlData.ButtonR1){
                ToolMotor1.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                digitalE.set(1);
            }
            if (ControlData.ButtonR2){
                ToolMotor1.spin(forward, TrackSpeed, voltageUnits::volt);
                digitalE.set(0);
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
                ToolMotor2.spin(forward, 3, voltageUnits::volt);
                // if ((ErrorBall1 > 0 && ErrorBall2 != 0) || ErrorBall2 > 0){
                //     ToolMotor2.spin(forward, 16, voltageUnits::volt);
                // }
                // else{
                //     ToolMotor2.spin(forward, 2, voltageUnits::volt);
                // }
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
        if (ControlData.ButtonL1 || ControlData.ButtonL2 || ControlData.ButtonR1){
            if (ControlData.ButtonL2){
                if (ErrorBall1 > 0){
                    ToolMotor3.spin(forward, TrackSpeed, voltageUnits::volt);
                }
                else{
                    ToolMotor3.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                }
            }
            if (ControlData.ButtonL1){
                if ((ErrorBall1 > 0 && ErrorBall2 != 0)  || ErrorBall2 > 0 ){
                    ToolMotor3.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                }
                else{
                    ToolMotor3.spin(forward, TrackSpeed, voltageUnits::volt);
                }
            }
            if (ControlData.ButtonR1){
                // ToolMotor3.spin(forward, 1.7, voltageUnits::volt);
                if ((ErrorBall1 > 0 && ErrorBall2 != 0)  || ErrorBall2 > 0 ){
                    ToolMotor3.spin(forward, -16, voltageUnits::volt);
                }
                else{
                    ToolMotor3.spin(forward, 1.7, voltageUnits::volt);
                }
            }
        }
        else{
            ToolMotor3.spin(forward, 0, voltageUnits::volt);
        }

        if (ControlData.ButtonX){
            digital_move_finish = false;
            digitalMod = 1;
            digitalDOverCount = 1;
            digitalGOverCount = 1;
        }
        if (ControlData.ButtonB){
            digital_move_finish = false;
            digitalMod = 0;
            digitalGOverCount = 1;
            digitalDOverCount = 1;
        }
        if (ControlData.ButtonDown){
            digitalH.set(1);
            digitalC.set(1);
            digitalG.set(0);
        }

        if (ControlData.ButtonUp && ! LastUp){
            CannonMod ^= 1;
            digitalB.set(CannonMod);
        }
        LastUp = ControlData.ButtonUp;

        if (digitalMod && digitalDOverCount != 0 && digitalGOverCount != 0){
            if (digitalDOverCount == 1){
              digitalC.set(0);
              digitalD.set(0);
            }
            
            digitalDOverCount += 1;
            if (digitalDOverCount >= digitalOverCount){
                digitalG.set(1);
                digitalDOverCount = 0;
            }
            
        }else if (!( digitalMod) && digitalGOverCount != 0 && digitalDOverCount != 0){
            digitalG.set(0);
            digitalC.set(0);
            digitalGOverCount += 1;
            if (digitalGOverCount >= digitalOverCount){

                digitalD.set(1);

                digitalDOverCount = 0;
            }
        }

        if (ControlData.ButtonY){
          digitalC.set(1);
          digitalD.set(0);
          digitalG.set(0);
        }

        if (ControlData.ButtonRight){
          digitalC.set(0);
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

        ErrorBall1 = std::max(ErrorBall1 - 1, 0);    // 反转计数器减小
        ErrorBall2 = std::max(ErrorBall2 - 1, 0);
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

// 低改高消分，双颜色传感器
void RemoteControl_2(bool SaveFile, bool ReadFile, int OutTime, std::string file_name, double timeout){
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
    int ErrorBall1 = 0;
    int ErrorBall2 = 0;

    // 大炮状态
    bool CannonMod = 0;
    bool LastUp    = 0;
    bool LastRight = 0;

    // 前后气动联动设置
    int digitalGOverCount = 0;
    int digitalDOverCount = 0;
    int digitalOverCount = 5;
    int digitalMod       = 1; // 1：前开后闭，0：前闭后开
    bool digital_move_finish = false;

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
        if (OpticalHaveRed(Optical1) || OpticalHaveBlue(Optical1)){
            HaveBall = 50;
        }else{
            HaveBall = 0;
        }

        if ((stance == 'r' && OpticalHaveBlue(Optical1)) || (stance == 'b' && OpticalHaveRed(Optical1))){
            ErrorBall1 = 20;
        }
        else if ((stance == 'b' && OpticalHaveBlue(Optical1)) || (stance == 'r' && OpticalHaveRed(Optical1))){
            ErrorBall1 = 0;
        }

        if ((stance == 'r' && OpticalHaveBlue(Optical2)) || (stance == 'b' && OpticalHaveRed(Optical2))){
            ErrorBall2 = 20;
        }
        else if ((stance == 'b' && OpticalHaveBlue(Optical2)) || (stance == 'r' && OpticalHaveRed(Optical2))){
            ErrorBall2 = 0;
        }
        else{
          ErrorBall2 = -1;
        }


        // R1 R2 -  吸球 吐球
        // L1 高层 吐
        // L2 中层 吐
        // A 分到后背

        // 1 号
        if (ControlData.ButtonL1 || ControlData.ButtonL2 || ControlData.ButtonR2 || ControlData.ButtonR1){
            if (ControlData.ButtonL2){
                ToolMotor1.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                digitalE.set(1);
            }
            if (ControlData.ButtonL1){
                ToolMotor1.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                digitalE.set(1);
            }

            if (ControlData.ButtonR1){
                ToolMotor1.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                digitalE.set(1);
            }
            if (ControlData.ButtonR2){
                ToolMotor1.spin(forward, TrackSpeed, voltageUnits::volt);
                digitalE.set(0);
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
                ToolMotor2.spin(forward, 3, voltageUnits::volt);
                // if ((ErrorBall1 > 0 && ErrorBall2 != 0) || ErrorBall2 > 0){
                //     ToolMotor2.spin(forward, 16, voltageUnits::volt);
                // }
                // else{
                //     ToolMotor2.spin(forward, 2, voltageUnits::volt);
                // }
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
        if (ControlData.ButtonL1 || ControlData.ButtonL2 || ControlData.ButtonR1){
            if (ControlData.ButtonL2){
                if (ErrorBall1 > 0){
                    ToolMotor3.spin(forward, TrackSpeed, voltageUnits::volt);
                }
                else{
                    ToolMotor3.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                }
            }
            if (ControlData.ButtonL1){
                if ((ErrorBall1 > 0 && ErrorBall2 != 0)  || ErrorBall2 > 0 ){
                    ToolMotor3.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                }
                else{
                    ToolMotor3.spin(forward, TrackSpeed, voltageUnits::volt);
                }
            }
            if (ControlData.ButtonR1){
                // ToolMotor3.spin(forward, 1.7, voltageUnits::volt);
                if ((ErrorBall1 > 0 && ErrorBall2 != 0)  || ErrorBall2 > 0 ){
                    ToolMotor3.spin(forward, -16, voltageUnits::volt);
                }
                else{
                    ToolMotor3.spin(forward, 1.7, voltageUnits::volt);
                }
            }
        }
        else{
            ToolMotor3.spin(forward, 0, voltageUnits::volt);
        }

        if (ControlData.ButtonX){
            digital_move_finish = false;
            digitalMod = 1;
            digitalDOverCount = 1;
            digitalGOverCount = 1;
        }
        if (ControlData.ButtonB){
            digital_move_finish = false;
            digitalMod = 0;
            digitalGOverCount = 1;
            digitalDOverCount = 1;
        }
        if (ControlData.ButtonDown){
            digitalG.set(0);
        }

        if (ControlData.ButtonUp && ! LastUp){
            CannonMod ^= 1;
            digitalB.set(CannonMod);
        }
        LastUp = ControlData.ButtonUp;

        if (digitalMod && digitalDOverCount != 0 && digitalGOverCount != 0){
            if (digitalDOverCount == 1){
              digitalD.set(0);
            }
            
            digitalDOverCount += 1;
            if (digitalDOverCount >= digitalOverCount){
                digitalG.set(1);
                digitalDOverCount = 0;
            }
            
        }else if (!( digitalMod) && digitalGOverCount != 0 && digitalDOverCount != 0){
            digitalG.set(0);
            digitalGOverCount += 1;
            if (digitalGOverCount >= digitalOverCount){

                digitalD.set(1);

                digitalDOverCount = 0;
            }
        }


        if (ControlData.ButtonRight && ! LastRight){
          digitalF.set(digitalF.value() ^ 1);
        }
        LastRight = ControlData.ButtonRight;



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

        ErrorBall1 = std::max(ErrorBall1 - 1, 0);    // 反转计数器减小
        ErrorBall2 = std::max(ErrorBall2 - 1, 0);
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


// 遥控额外线程控制
int RemoteControl_3_child(){
    bool LastRight = false;
    while(true){
        if (ControlData.ButtonRight && ! LastRight){
            digitalC.set(digitalC.value() ^ 1);
            wait(500, msec);
            digitalD.set(digitalC.value());
        }
        LastRight = ControlData.ButtonRight;
        wait(50, msec);
    }
}

// 二代车，双颜色传感器
void RemoteControl_3(bool SaveFile, bool ReadFile, int OutTime, std::string file_name, double timeout){

    // digitalG  铲子
    // digitalH  抬升
    // digitalF  撞球
    // digitalE  钩子的大架子
    // digitalD  左边钩子
    // digitalC  右边钩子


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

    // 退出计数
    int HoldButtonBCount = 0;

    double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); 
    double now_time = start_time; 
    double out_time = 0;
    double last_time = start_time; 

    bool LastButtonX = false;

    vex::task Task(RemoteControl_3_child);  // 线程开启

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
        if (OpticalHaveRed(Optical1) || OpticalHaveBlue(Optical1)){
            HaveBall = 50;
        }else{
            HaveBall = 0;
        }

        if ((stance == 'r' && OpticalHaveBlue(Optical1)) || (stance == 'b' && OpticalHaveRed(Optical1))){
            ErrorBall = 20;
        }
        else if ((stance == 'b' && OpticalHaveBlue(Optical1)) || (stance == 'r' && OpticalHaveRed(Optical1))){
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
                digitalF.set(0);
                digitalE.set(1);
            }
            if (ControlData.ButtonL1){
                ToolMotor1.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
                digitalH.set(1);
                digitalF.set(0);
                digitalE.set(1);
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

            if (ControlData.ButtonR2){
                ToolMotor2.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
            }

        }
        else{
            ToolMotor2.spin(forward, 0, voltageUnits::volt);
        }

        // 3号
        if (ControlData.ButtonL1 || ControlData.ButtonL2 || ControlData.ButtonR1){
            if (ControlData.ButtonL2){
                ToolMotor3.spin(forward, -1 * TrackSpeed, voltageUnits::volt);
            }
            if (ControlData.ButtonL1){
                ToolMotor3.spin(forward, TrackSpeed, voltageUnits::volt);
            }
            if (ControlData.ButtonR1){
                ToolMotor3.spin(forward, 1.7, voltageUnits::volt);
            }
        }
        else{
            ToolMotor3.spin(forward, 0, voltageUnits::volt);
        }

        if (ControlData.ButtonX && ! LastButtonX){
            digitalF.set(digitalF.value() ^ 1);
        }
        LastButtonX = ControlData.ButtonX;

        
        if (ControlData.ButtonDown){
            digitalC.set(0);
            digitalD.set(0);
            digitalE.set(0);
            digitalF.set(1);
            digitalH.set(0);
        }

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
}

// 整体遥控
void RemoteControlUnit(bool SaveFile, bool ReadFile, int OutTime, std::string file_name, double timeout){

    // RemoteControl_1(SaveFile, ReadFile, OutTime, file_name, timeout);   // 消分一代车遥控，双颜色传感器分球
    // RemoteControl_2(SaveFile, ReadFile, OutTime, file_name, timeout);  // 低改高消分，双颜色传感器
    RemoteControl_3(SaveFile, ReadFile, OutTime, file_name, timeout);  // 二代车，双颜色传感器

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