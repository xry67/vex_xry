#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"

// 左0+4 横向向左摆位，记得回收钩子
void auto5(){
    ChassisInertial.setRotation(0, degrees);
    ChassisControlSTS.Over_count = 1;
    // 第一个导入桶
    digitalC.set(1);
    WaitFrontDistanceArriveThenStopMove(12, 1690, 0, 425);
    digitalG.set(1);
    wait(100,msec);   // 需要保留等待时间，否则自转的旋转中心会由于惯性发生改变
    intake_and_pushback(-90, 44, 500);
    ChassisControlSTS.SetStopModCoast();
    ToolMotor2.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, 16, voltageUnits::volt);
    ToolMotor3.spin(forward, 16, voltageUnits::volt);
    wait(900, msec);

    // 钩子
    digitalC.set(0);
    ChassisControlSTS.TurnTo(10, -90, false);
    ToolMotor2.spin(forward, 0, voltageUnits::volt);
    ToolMotor3.spin(forward, 0, voltageUnits::volt);
    ChassisControlSTS.Move_PPS = {0.015, 0, 0};
    ChassisControlSTS.CurveMove(3, 292, 900, 'q');   // 曲线

    ChassisControlSTS.Move(12, 1700, -90-180, 'n', true); // 直线钩
    while (true){
      // 如果距离或者时间到了，都要退出
      if (ChassisControlSTS.GetPositions_mean() >= 1500 || RunningAutoFun->GetAutoRunTime() >= 14.5){
        break;
      }
      wait(10, msec);
    }
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
    wait(15000, msec);

}