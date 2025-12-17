#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "RobotControl.h"
#include "RemoteControl.h"
#include "AutoTools.h"

void auto12(){
    double best_vertical_d = 620; // 竖直方向，接近墙壁距离
    double best_horizontal_d = 475; // 水平方向，接近墙壁距离
    double uniform_start_end_d = 0; // 恒力开始距离
    double uniform_speed_end_d = 0; // 恒力结束距离
    double best_intake_d = 0; // 导入最佳距离

    ChassisInertial.setRotation(0, degrees);

    // 第一个导入桶
    WaitFrontDistanceArriveThenStopMove(12, 1690, 0, best_horizontal_d);
    digitalG.set(1);
    wait(100,msec);   // 需要保留等待时间，否则自转的旋转中心会由于惯性发生改变
    intake_and_pushback(90, 200, 3000);
    ChassisControlSTS.Move(12, 350, 90, 'n');        
    ChassisControlSTS.TurnTo(12, 180, false);
    ChassisControlSTS.Move(12, 639, 'n');
    ChassisControlSTS.TurnTo(12, 270, false);


    // 第二个导入桶
    ToolMotor2.spin(forward, 10, pct);  // 递球
    ToolMotor3.spin(forward, -3, voltageUnits::volt);

    WaitFrontDistanceArriveThenStopMove(16, 4061, 270, best_vertical_d);
    wait(200,msec);
    ChassisControlSTS.TurnTo(12, 360, false);
    WaitFrontDistanceArriveThenStopMove(3, 319, 360, best_horizontal_d);
    digitalG.set(1);
    wait(100,msec);   // 需要保留等待时间，否则自转的旋转中心会由于惯性发生改变
    intake_and_pushback(270, 200, 3000);
    ChassisControlSTS.Move(12, 450, 270, 'n');        
    ChassisControlSTS.TurnTo(12, 180, false);

    // 第三个导入桶
    ToolMotor2.spin(forward, 20, pct);  // 递球
    ToolMotor3.spin(forward, -3, voltageUnits::volt);
    
    WaitFrontDistanceArriveThenStopMove(16, 4160, 180, best_horizontal_d);
    digitalG.set(1);
    wait(100,msec);   // 需要保留等待时间，否则自转的旋转中心会由于惯性发生改变
    intake_and_pushback(270, 200, 3000);
    ChassisControlSTS.Move(12, 350, 270, 'n');  
    ChassisControlSTS.TurnTo(12, 0, false);
    ChassisControlSTS.Move(12, 669, 'n');
    ChassisControlSTS.TurnTo(12, 90, false); // 实际角度 360 + 90

    // 第四个导入桶
    ToolMotor2.spin(forward, 20, pct);  // 递球
    ToolMotor3.spin(forward, -3, voltageUnits::volt);

    WaitFrontDistanceArriveThenStopMove(16, 4061, 360 + 90, best_vertical_d);
    wait(200,msec);
    ChassisControlSTS.TurnTo(12, 180, false);
    WaitFrontDistanceArriveThenStopMove(3, 259, 360 + 180, best_horizontal_d);
    digitalG.set(1);
    wait(100,msec);   // 需要保留等待时间，否则自转的旋转中心会由于惯性发生改变
    intake_and_pushback(360 + 90, 200, 3000);
    
    ChassisControlSTS.Move(12, 350, 360 + 90, 'n');        
    ChassisControlSTS.Move(8, 2000, 390, 'n', true);
    ChassisControlSTS.WaitPositionArrive(1000);
    ChassisControlSTS.ResetStraightAngle(360);
    ChassisControlSTS.SetStopModCoast();
    ChassisControlSTS.WaitPositionArrive(1800);
    ChassisControlSTS.StopMoving();

    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, 100, pct);  // 递球
    ToolMotor3.spin(forward, 16, voltageUnits::volt);
    ChassisControlSTS.ChassisLeftControl(-8);
    ChassisControlSTS.ChassisRightControl(-8);
    wait(250, msec);
    ChassisControlSTS.SetStopModHold();
    ChassisControlSTS.ChassisLeftControl(1);
    ChassisControlSTS.ChassisRightControl(1);
    wait(100, msec);
    ChassisControlSTS.ChassisLeftControl_pct(0);
    ChassisControlSTS.ChassisRightControl_pct(0);
    wait(500, msec);

    ChassisControlSTS.ChassisLeftControl(8);
    ChassisControlSTS.ChassisRightControl(8);
    wait(1600, msec);
    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);
    wait(1000, msec);
}

// void auto12(){

//     vex::thread  Task = thread(auto12_child);

//     double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
//     double end_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间

//     while (end_time - start_time < 60){
//         end_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();  // 运动开始时间
//         if (Controller.ButtonB.pressing() && Controller.ButtonA.pressing()){
//             break;
//         }
//         wait(10, msec); 
//     }
//     Task.interrupt(); // 中止线程
//     RestoreInitial(); // 停止所有
//     RemoteControl();
// }   

