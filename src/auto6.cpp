#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

// 距离传感器测试（前）
void auto6(){
    for (int l=2000; l>100; l-=500){
        ChassisControlSTS.MoveToDistance(16, l, "front");
        ChassisControlSTS.MoveToDistance(16, 2000, "front");
    }
    ChassisControlSTS.MoveToDistance(16, 200, "front");
    ChassisControlSTS.MoveToDistance(16, 2000, "front");
}