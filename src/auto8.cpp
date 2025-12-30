#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"

// 右0+7+钩 队牌摆位，记得回收钩子
void auto8(){
    ChassisControlSTS.Over_count = 1;
    ChassisInertial.setRotation(22, deg);
    SimulatedRemoteControl("R1");

    // 前进吸球
    ChassisControlSTS.Move(10, 1400, 'n', true); 

    ChassisControlSTS.WaitPositionArrive(400);
    ChassisControlSTS.ResetMaxSpeed(3);
    ChassisControlSTS.WaitPositionArrive(1000);
    digitalH.set(1);
    ChassisControlSTS.WaitMoveFinish();

    ChassisControlSTS.AutoPPSTurnTo(16, -25);
    // 高阶==================================================
    // 斜线到达判定距离传感器位置
    ChassisControlSTS.Move(12, 2000, -17, 'u', true);        
    ChassisControlSTS.WaitPositionArrive(800);

    // 重新对齐至-90度
    ChassisControlSTS.ResetMaxSpeed(2);        
    ChassisControlSTS.ResetStraightAngle(-90);
    ChassisControlSTS.WaitPositionArrive(1100);

    // 等待到达指定距离
    while (GetDistance_back() > 455){
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
    wait(200,msec);   // 需要保留等待时间，否则自转的旋转中心会由于惯性发生改变

    digitalH.set(1);

    // 转至180
    ChassisControlSTS.Over_count = 5;
    ChassisControlSTS.AutoPPSTurnTo(10, 180);
    ChassisControlSTS.Over_count = 1;

    intake_and_pushback_new(-180, 450, 2000);

    // 钩子
    Hook(-180);
}