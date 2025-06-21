#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

// 曲线测试
void auto11(){

    ChassisControlSTS.MoveBySmallWheel(5, 1500, 0.1, 'p');

}