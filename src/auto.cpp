#include "vex.h"
#include "robot-config.h"
#include "auto.h"

// 自动函数增加
void AddAutoFun()
{
    AutoFunctions.push_back(AutoFun("测试", auto1, 99999999999999999));
}

// 比赛的自动函数
void AutoControl()
{
    // 运行自动程序
    AutoFunctions[auto_index].RunAutoFun();
}
