#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

// 距离传感器测试（后）
void auto7(){
    for (int l=1500; l>100; l-=500){
        ChassisControlSTS.MoveToDistance(7, l, "back");
        ChassisControlSTS.MoveToDistance(7, 1500, "back");
    }
    ChassisControlSTS.MoveToDistance(7, 200, "back");
    ChassisControlSTS.MoveToDistance(7, 1500, "back");
}