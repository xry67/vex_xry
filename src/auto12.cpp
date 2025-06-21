#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

// 
void auto12(){
    ChassisControlSTS.Move(7, 3000, 0.0, 'n', true);  // 直行

    wait(500, msec);  // 等待
    // 进行一一些其他操作
    // 比如打开吸球系统
    ChassisControlSTS.WaitMoveFinish();  // 等待直行完成

    ChassisControlSTS.Turn(7, 90, 'l');  // 自转90度

}

