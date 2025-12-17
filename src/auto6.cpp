#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"

// 右0+4 横向向右摆位，记得回收钩子
void auto6(){
    ChassisInertial.setRotation(0, degrees);
    ChassisControlSTS.Over_count = 1;
    // 第一个导入桶
    WaitFrontDistanceArriveThenStopMove(12, 1690, 0, 435);
    digitalH.set(1);
    wait(100,msec);   // 需要保留等待时间，否则自转的旋转中心会由于惯性发生改变
    // 转至180
    ChassisControlSTS.AutoPPSTurnTo(10, 90);

    // 吸导入
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, 10, pct);
    // ToolMotor3.spin(forward, 25, pct);
        // 运球

    ChassisControlSTS.Move(5, 900, 90, 'n', true); // 1550  

    // 使用距离传感器判断是否到达恒力区域
    while (GetDistance_front() > 270){
      wait(10, msec);
    }
    ChassisControlSTS.SetStopModCoast();   // 需要保留，否则容易插不进去
    ChassisControlSTS.StopMoving();
    wait(100,msec);

    AutoIntake();

    // 恒力吸导入  pct可以保证匀速插入导入筒
    for (int i=0; i<200; i++){
      ChassisControlSTS.ChassisLeftControl_pct(2);
      ChassisControlSTS.ChassisRightControl_pct(2);
      if (GetDistance_front() < 250){
        break;
      }
      wait(10, msec);
    }

    // 轻微的蠕动
    for (int i=0; i<50; i++){
      if (GetDistance_front() > 137){
        ChassisControlSTS.ChassisLeftControl(2);
        ChassisControlSTS.ChassisRightControl(2);
      }else{
        ChassisControlSTS.ChassisLeftControl(0);
        ChassisControlSTS.ChassisRightControl(0);
      }
      if ((stance == 'r' && OpticalHaveBlue(Optical1)) || (stance == 'b' && OpticalHaveRed(Optical1))){
        break;
      }
      wait(10, msec);
    }

    StopAutoIntake();

    // 倒车 打球
    ChassisControlSTS.Move(7, 1500, 90, 'u', true); // 1550
    ChassisControlSTS.WaitPositionArrive(500);
    // 运球
    ToolMotor2.spin(forward, 10, pct);
    ToolMotor3.spin(forward, 25, pct);
    ToolMotor1.spin(forward, -2, voltageUnits::volt);

    // 提前打球
    ChassisControlSTS.WaitPositionArrive(1200);
    digitalH.set(0);

    // 打球
    ToolMotor1.spin(forward, 16, voltageUnits::volt);
    ToolMotor2.spin(forward, 2, voltageUnits::volt);
    ToolMotor3.spin(forward, 16, voltageUnits::volt);

    ChassisControlSTS.SetStopModCoast();
    ChassisControlSTS.StopMoving();
    // 努力进入对位
    ChassisControlSTS.ChassisLeftControl(-2);
    ChassisControlSTS.ChassisRightControl(-2);
    wait(1000, msec);

    // 钩子
    ChassisControlSTS.Move_PPS = {0.0182, 0.5, 2};
    ChassisControlSTS.maxinum_speed_acceleration = 0.5;
    ChassisControlSTS.Move(16, 400, 'n'); // 1550

    ChassisControlSTS.maxinum_speed_acceleration_turn = 0.1;
    ChassisControlSTS.AutoPPSTurnTo(10, 40);

    ChassisControlSTS.CurveMove(10, 680, 850, 'd', true);
    ChassisControlSTS.WaitPositionArrive(750);
    ChassisControlSTS.StopMoving();

    ChassisControlSTS.Move(10, 1000, -180, 'u', true); // 1550
    ChassisControlSTS.WaitPositionArrive(550);
    ChassisControlSTS.SetStopModHold();
    ChassisControlSTS.StopMoving();
    ToolMotor1.spin(forward, 0, voltageUnits::volt);
    ToolMotor2.spin(forward, 0, voltageUnits::volt);
    ToolMotor3.spin(forward, 0, voltageUnits::volt);
    // 急刹车
    ChassisControlSTS.ChassisLeftControl(-6);
    ChassisControlSTS.ChassisRightControl(6);
    wait(500, msec);
    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);
    wait(10000, msec);
}