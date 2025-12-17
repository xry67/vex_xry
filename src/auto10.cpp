#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"


// 右6+1+钩 队牌摆位，记得回收钩子
void auto10(){
    double k=1;
    ChassisInertial.setRotation(0, degrees);
    ChassisControlSTS.Over_count = 5;
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ChassisControlSTS.CurveMove(10, 780, 710, 'p', true);
    ChassisControlSTS.WaitPositionArrive(680);
    ChassisControlSTS.StopMoving();

    ChassisControlSTS.Move_PPS = {0.0150, 0.500, 2};
    ChassisControlSTS.Move(4, 590, 'n', true); // 1550
    ChassisControlSTS.WaitPositionArrive(570);
    ChassisControlSTS.StopMoving();

    ChassisControlSTS.Over_count = 10;
    ChassisControlSTS.TurnTo(14, 360  - 48, false); // 1550
    ChassisControlSTS.Over_count = 5;

    digitalG.set(0);
    ChassisControlSTS.Move(16, 450, 'n', true); // 1550
    ChassisControlSTS.WaitPositionArrive(420);
    ToolMotor2.spin(forward, -16, voltageUnits::volt);
    ToolMotor1.spin(forward, 16, voltageUnits::volt);
    wait(1000, msec);
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, 0, voltageUnits::volt);
    ChassisControlSTS.WaitMoveFinish();
    digitalC.set(1);
    digitalG.set(0);


    // 高阶============================================================================
    // 斜线到达判定距离传感器位置
    ChassisControlSTS.Move(12, 2400, 'u', true);        
    ChassisControlSTS.WaitPositionArrive(1100);

    // 重新对齐至-90度
    ChassisControlSTS.ResetMaxSpeed(2);        
    ChassisControlSTS.ResetStraightAngle(-90);
    ChassisControlSTS.WaitPositionArrive(1400);

    // 等待到达指定距离
    while (GetDistance_back() > 505){
      std::cout << GetDistance_back() << std::endl;
      if (! ChassisControlSTS.IsMoving()){
          ChassisControlSTS.ChassisLeftControl(2);
          ChassisControlSTS.ChassisRightControl(2);
      }
      wait(10, msec);
    }
    
    // 刹车
    digitalG.set(1);
    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);
    ChassisControlSTS.StopMoving();
    wait(100,msec);   // 需要保留等待时间，否则自转的旋转中心会由于惯性发生改变
    
    // 转至180
    ChassisControlSTS.Over_count = 10;
    ChassisControlSTS.TurnTo(10, 180, false);
    ChassisControlSTS.Over_count = 5;

    // 吸导入
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, 50, pct);
    ToolMotor3.spin(forward, 3, voltageUnits::volt);
    ChassisControlSTS.Move(5, 900, 'n', true); // 1550  

    // 使用距离传感器判断是否到达恒力区域 250是即将进入导入区的距离
    while (GetDistance_front() > 300){
      std::cout << GetDistance_front() << std::endl;
      wait(10, msec);
    }
    ChassisControlSTS.SetStopModCoast();   // 需要保留，否则容易插不进去
    ChassisControlSTS.StopMoving();
    wait(100,msec);

    // 恒力吸导入  pct可以保证匀速插入导入筒
    for (int i=0; i<200; i++){
      ChassisControlSTS.ChassisLeftControl_pct(2);
      ChassisControlSTS.ChassisRightControl_pct(2);
      if (GetDistance_front() < 276){
        break;
      }
      wait(10, msec);
    }

    // 轻微的蠕动
    for (int i=0; i<45; i++){
      if (GetDistance_front() > 145){
        ChassisControlSTS.ChassisLeftControl(1.5);
        ChassisControlSTS.ChassisRightControl(1.5);
      }else{
        ChassisControlSTS.ChassisLeftControl(0);
        ChassisControlSTS.ChassisRightControl(0);
      }
      if ((stance == 'r' && OpticalHaveBlue(Optical1)) || (stance == 'b' && OpticalHaveRed(Optical1))){
        break;
      }
      wait(10, msec);
    }

    // 倒车 打球
    ChassisControlSTS.Move(7, 1500, -180, 'u', true); // 1550
    ChassisControlSTS.WaitPositionArrive(500);
    ToolMotor2.spin(forward, 20, pct);

    // 提前打球
    ChassisControlSTS.WaitPositionArrive(1100);
    ToolMotor2.spin(forward, 16, voltageUnits::volt);
    ToolMotor3.spin(forward, 16, voltageUnits::volt);
    digitalG.set(0);

    ChassisControlSTS.SetStopModCoast();
    ChassisControlSTS.StopMoving();
    // 努力进入对位
    ChassisControlSTS.ChassisLeftControl(-3);
    ChassisControlSTS.ChassisRightControl(-3);
    wait(1000, msec);


    // 钩子
    digitalC.set(0);
    ChassisControlSTS.TurnTo(10, 180, false);
    ToolMotor2.spin(forward, 0, voltageUnits::volt);
    ToolMotor3.spin(forward, 0, voltageUnits::volt);
    ChassisControlSTS.Move_PPS = {0.015, 0, 0};
    ChassisControlSTS.CurveMove(3, 292, 900, 'q');   // 曲线

    ChassisControlSTS.Move(12, 1700, -360, 'n', true); // 直线钩
    ChassisControlSTS.WaitPositionArrive(1500);
    ChassisControlSTS.SetStopModHold();

    ChassisControlSTS.StopMoving();
    ToolMotor1.spin(forward, 0, voltageUnits::volt);
    ToolMotor2.spin(forward, 0, voltageUnits::volt);
    ToolMotor3.spin(forward, 0, voltageUnits::volt);
    // 急刹车
    ChassisControlSTS.ChassisLeftControl(-2);
    ChassisControlSTS.ChassisRightControl(-2);
    wait(100, msec);
    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);
    ChassisControlSTS.ChassisLeftControl(16);
    ChassisControlSTS.ChassisRightControl(-16);
    wait(300, msec);
    ChassisControlSTS.ChassisLeftControl_pct(0);
    ChassisControlSTS.ChassisRightControl_pct(0);
    wait(3000, msec);
}