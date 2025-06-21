#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

// 旋转（定位目标）（柱子）
void auto8(){
    ChassisControlSTS.Target.SetColor(Orange);
    // ChassisControlSTS.Target.Limit_w = {999, 999};
    // ChassisControlSTS.Target.Limit_x = {0, aiVision_w / 2};
    ChassisControlSTS.TurnToTarget(7, 90, 'l');
}