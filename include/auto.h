#ifndef AUTO_H  
#define AUTO_H 

#include "vex.h"
#include "robot-config.h"

void auto1();
void auto2();


// 自动函数增加
void AddAutoFun(){
    AutoFunctions.push_back(AutoFun("自动程序1", auto1));
    AutoFunctions.push_back(AutoFun("自动程序2", auto2));
}

#endif 