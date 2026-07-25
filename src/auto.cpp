#include "vex.h"
#include "robot-config.h"
#include "auto.h"

// 自动函数增加
void AddAutoFun()
{
    AutoFunctions.push_back(AutoFun("侧扇区-近", auto1, 15));
    AutoFunctions.push_back(AutoFun("侧扇区-远", auto2, 15));
    // AutoFunctions.push_back(AutoFun("操作区-近", auto3, 15));
    // AutoFunctions.push_back(AutoFun("操作区-远", auto4, 15));
}

// 比赛的自动函数
void AutoControl()
{
    // 运行自动程序
    AutoFunctions[auto_index].RunAutoFun();
}
