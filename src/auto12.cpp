#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"
#include "RobotControl.h"
#include "RemoteControl.h"


void intake_and_pushback(double angle){


    // 转至180
    ChassisControlSTS.TurnTo(10, NormsAngle_0_360(angle), false);

    // 吸导入
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, 15, pct);  // 递球
    ToolMotor3.spin(forward, 3, voltageUnits::volt);
    ChassisControlSTS.Move(5, 900, angle, 'n', true); // 1550  

    // 使用距离传感器判断是否到达恒力区域 250是即将进入导入区的距离
    while (GetDistance_front() > 300){
      std::cout << GetDistance_front() << std::endl;
      wait(10, msec);
    }
    ChassisControlSTS.SetStopModCoast();   // 需要保留，否则容易插不进去
    ChassisControlSTS.StopMoving();
    wait(100,msec);

    // 恒力吸导入  pct可以保证匀速插入导入筒
    while(GetDistance_front() > 200){
      ChassisControlSTS.ChassisLeftControl_pct(2);
      ChassisControlSTS.ChassisRightControl_pct(2);
    }

    // 轻微的蠕动
    for (int i=0; i<200; i++){
      if (GetDistance_front() > 172){
        ChassisControlSTS.ChassisLeftControl(1.5);
        ChassisControlSTS.ChassisRightControl(1.5);
      }else{
        ChassisControlSTS.ChassisLeftControl(0);
        ChassisControlSTS.ChassisRightControl(0);
      }
      wait(10, msec);
    }

    // 倒车 打球
    ChassisControlSTS.Move(7, 1500, angle, 'u', true); // 1550
    ToolMotor2.spin(forward, 20, pct);  // 递球
    ToolMotor3.spin(forward, 3, voltageUnits::volt);
    digitalG.set(0);

    // 提前打球
    ChassisControlSTS.WaitPositionArrive(1100);
    ToolMotor2.spin(forward, 16, voltageUnits::volt);
    ToolMotor3.spin(forward, 16, voltageUnits::volt);
    ChassisControlSTS.SetStopModCoast();
    ChassisControlSTS.StopMoving();
    // 努力进入对位
    ChassisControlSTS.ChassisLeftControl(-3);
    ChassisControlSTS.ChassisRightControl(-3);
    wait(3000, msec);
    ChassisControlSTS.SetStopModHold();
    ToolMotor2.spin(forward, 0, voltageUnits::volt);
    ToolMotor3.spin(forward, 0, voltageUnits::volt);
}

// 等待前方距离传感器到达某个时间，然后停止运动
void WaitFrontDistanceArriveThenStopMove(double speed, double val, double angle, double Distance){
    
    ChassisControlSTS.Move(speed, val, angle, 'n', true);
    // 等待到达指定距离
    while (GetDistance_front() > Distance){
      if (! ChassisControlSTS.IsMoving()){
          ChassisControlSTS.ChassisLeftControl(2);
          ChassisControlSTS.ChassisRightControl(2);
      }
      wait(10, msec);
    }
    // 刹车
    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);
    ChassisControlSTS.StopMoving();
}

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
    intake_and_pushback(90);
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
    intake_and_pushback(270);
    ChassisControlSTS.Move(12, 450, 270, 'n');        
    ChassisControlSTS.TurnTo(12, 180, false);

    // 第三个导入桶
    ToolMotor2.spin(forward, 20, pct);  // 递球
    ToolMotor3.spin(forward, -3, voltageUnits::volt);
    
    WaitFrontDistanceArriveThenStopMove(16, 4160, 180, best_horizontal_d);
    digitalG.set(1);
    wait(100,msec);   // 需要保留等待时间，否则自转的旋转中心会由于惯性发生改变
    intake_and_pushback(270);
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
    intake_and_pushback(360 + 90);
    
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

