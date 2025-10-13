#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"


void auto8(){
    // 斜线到达判定距离传感器位置
    ChassisControlSTS.Move(12, 2800, 'n', true); // 1550        
    ChassisControlSTS.WaitPositionArrive(1300);

    // 重新对其至-90度
    ChassisControlSTS.ResetMaxSpeed(2);        
    ChassisControlSTS.ResetStraightAngle(-90);
    ChassisControlSTS.WaitPositionArrive(1600);

    // 等待到达指定距离
    while (GetDistance_front() > 400){
      std::cout << GetDistance_front() << std::endl;
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

    // 使用距离传感器判断是否到达恒力区域
    while (GetDistance_front() > 250){
      std::cout << GetDistance_front() << std::endl;
      wait(10, msec);
    }
    ChassisControlSTS.SetStopModCoast();
    ChassisControlSTS.StopMoving();
    wait(100,msec);

    // 恒力吸导入
    while(GetDistance_front() > 170){
      ChassisControlSTS.ChassisLeftControl(1.5);
      ChassisControlSTS.ChassisRightControl(1.5);
    }

    wait(500, msec);

    ChassisControlSTS.Move(7, 1500, -180, 'u', true); // 1550
    ToolMotor2.spin(forward, 16, voltageUnits::volt);
    ToolMotor3.spin(forward, 3, voltageUnits::volt);
    digitalH.set(1);  
    digitalG.set(0);

    ChassisControlSTS.WaitPositionArrive(1100);

    ToolMotor2.spin(forward, 16, voltageUnits::volt);
    ToolMotor3.spin(forward, 16, voltageUnits::volt);
    ChassisControlSTS.StopMoving();
    ChassisControlSTS.ChassisLeftControl(-2);
    ChassisControlSTS.ChassisRightControl(-2);
    wait(1000, msec);

    digitalC.set(1);
    digitalD.set(1);
    digitalE.set(1);
    ChassisControlSTS.SetStopModCoast();
    ChassisControlSTS.CurveMove(3, 290, 1500, 'p', true);
    ChassisControlSTS.WaitPositionArrive(1070);
    ChassisControlSTS.StopMoving();

    digitalC.set(0);
    digitalD.set(0);
    ChassisControlSTS.Move(12, 1500, 0, 'n', true); // 1550
    ChassisControlSTS.WaitPositionArrive(1300);
    ChassisControlSTS.SetStopModHold();

    ChassisControlSTS.StopMoving();

    ChassisControlSTS.ChassisLeftControl(-2);
    ChassisControlSTS.ChassisRightControl(-2);
    wait(100, msec);
    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);


}