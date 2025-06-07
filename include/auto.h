#ifndef AUTO_H  
#define AUTO_H 

#include "vex.h"
#include "robot-config.h"

// 自动函数声明
void auto1();
void auto2();
void auto3();
void auto4();
void auto5();
void auto6();
void auto7();
void auto8();
void auto9();
void auto10();
void auto11();
void auto12();

// 自动函数增加
void AddAutoFun(){
    AutoFunctions.push_back(AutoFun("自动程序1", auto1));
    AutoFunctions.push_back(AutoFun("自动程序2", auto2));
    AutoFunctions.push_back(AutoFun("测试", auto3));
}

#endif 