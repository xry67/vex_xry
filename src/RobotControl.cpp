#include <vector>
#include "vex.h"
#include "robot-config.h"

// 指定电机旋转到指定位置
void MotorSpinFor(motor Motor, double v, double val, bool task){
    // Motor: 电机名称
    // v    : 速度
    // val  : 旋转位置
    // task : 多线程
    Motor.spinFor(forward, v, degrees, val, dps, ! task);
}

