#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"


// 右6+1+钩 队牌摆位
void auto10(){
    double k=1;
    ChassisInertial.setRotation(0, degrees);
    ChassisControlSTS.Over_count = 5;
    ChassisControlSTS.CurveMove(10, 780, 700, 'p');

    ToolMotor1.spin(forward, -16, voltageUnits::volt);

    ChassisControlSTS.Move(4, 570 * k, 'n'); // 1550

    ChassisControlSTS.TurnTo(14, 360  - 40, false); // 1550

    ChassisControlSTS.Move(16, 560 * k, 'n', true); // 1550
    ChassisControlSTS.WaitPositionArrive(500);
    ToolMotor2.spin(forward, -16, voltageUnits::volt);
    ToolMotor1.spin(forward, 16, voltageUnits::volt);
    wait(500, msec);
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, 0, voltageUnits::volt);
    ChassisControlSTS.WaitMoveFinish();
    digitalH.set(1);
    digitalE.set(1);

    // 高阶============================================================================
    // 斜线到达判定距离传感器位置
    ChassisControlSTS.Move(12, 2400, 'u', true);        
    ChassisControlSTS.WaitPositionArrive(1100);

    // 重新对齐至-90度
    ChassisControlSTS.ResetMaxSpeed(2);        
    ChassisControlSTS.ResetStraightAngle(-90);
    ChassisControlSTS.WaitPositionArrive(1400);

    // 等待到达指定距离
    while (GetDistance_back() > 495){
      std::cout << GetDistance_back() << std::endl;
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
    wait(100,msec);   // 需要保留等待时间，否则自转的旋转中心会由于惯性发生改变
    digitalG.set(1);

    // 转至180
    ChassisControlSTS.TurnTo(10, 180, false);

    // 吸导入
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ChassisControlSTS.Move(5, 900, 'n', true); // 1550  

    // 使用距离传感器判断是否到达恒力区域 250是即将进入导入区的距离
    while (GetDistance_front() > 250){
      std::cout << GetDistance_front() << std::endl;
      wait(10, msec);
    }
    ChassisControlSTS.SetStopModCoast();   // 需要保留，否则容易插不进去
    ChassisControlSTS.StopMoving();
    wait(100,msec);

    // 恒力吸导入  pct可以保证匀速插入导入筒
    while(GetDistance_front() > 120){
      ChassisControlSTS.ChassisLeftControl_pct(2);
      ChassisControlSTS.ChassisRightControl_pct(2);
    }

    // 轻微的蠕动
    for (int i=0; i<50; i++){
      if (GetDistance_front() > 118){
        ChassisControlSTS.ChassisLeftControl(1.5);
        ChassisControlSTS.ChassisRightControl(1.5);
      }else{
        ChassisControlSTS.ChassisLeftControl(0);
        ChassisControlSTS.ChassisRightControl(0);
      }
      wait(10, msec);
    }

    // 倒车 打球
    ChassisControlSTS.Move(7, 1500, -180, 'u', true); // 1550
    ToolMotor2.spin(forward, 20, pct);  // 递球
    ToolMotor3.spin(forward, 3, voltageUnits::volt);
    digitalH.set(1);  
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
    wait(1500, msec);


    // 钩子
    ChassisControlSTS.CurveMove(3, 295, 900, 'q');   // 曲线

    ChassisControlSTS.Move(12, 1700, -360, 'n', true); // 直线钩
    ChassisControlSTS.WaitPositionArrive(1500);
    ChassisControlSTS.SetStopModHold();

    ChassisControlSTS.StopMoving();
    // 急刹车
    ChassisControlSTS.ChassisLeftControl(-2);
    ChassisControlSTS.ChassisRightControl(-2);
    wait(100, msec);
    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);
}