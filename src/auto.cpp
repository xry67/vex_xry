#include "vex.h"
#include "robot-config.h"
#include "auto.h"

// 自动函数增加
void AddAutoFun()
{
    // AutoFunctions.push_back(AutoFun("直走最小速度", auto1));
    // AutoFunctions.push_back(AutoFun("转向最小速度", auto2));
    // AutoFunctions.push_back(AutoFun("测试", auto3, 9999999999999));  // 无距离传感器
    // AutoFunctions.push_back(AutoFun("钩子", auto4));  // 无距离传感器
    // AutoFunctions.push_back(AutoFun("左0+4+钩", auto5));
    // AutoFunctions.push_back(AutoFun("右0+4+钩", auto6));
    // AutoFunctions.push_back(AutoFun("矮4+中3", auto7));
    // AutoFunctions.push_back(AutoFun("右0+7+钩", auto8));
    // AutoFunctions.push_back(AutoFun("左3+4+钩", auto9)); // 有距离传感器
    // AutoFunctions.push_back(AutoFun("右3+4+钩", auto10)); // 有距离传感器
    // AutoFunctions.push_back(AutoFun("钩子", auto11));
    // AutoFunctions.push_back(AutoFun("技能赛", auto12, 60));
    AutoFunctions.push_back(AutoFun("测试", auto13, 999999999));
    AutoFunctions.push_back(AutoFun("//", auto14));
}

// 比赛的自动函数
void AutoControl()
{
    // 运行自动程序
    AutoFunctions[auto_index].RunAutoFun();
}
