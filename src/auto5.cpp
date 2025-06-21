#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"


// 电机编码器测试
void auto5(){
    for (int l=2000; l>100; l-=500){
        ChassisControlSTS.Move(7, l, 0, 'n');
        ChassisControlSTS.Move(7, l, 0, 'u');
    }
    ChassisControlSTS.Move(7, 200, 0, 'n');
    ChassisControlSTS.Move(7, 200, 0, 'u');
}