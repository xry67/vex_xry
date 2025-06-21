#include "vex.h"
#include "robot-config.h"
#include "auto.h"

// 自动函数增加
void AddAutoFun(){
    AutoFunctions.push_back(AutoFun("自动程序1", auto1));
    AutoFunctions.push_back(AutoFun("自动程序2", auto10));
    AutoFunctions.push_back(AutoFun("相机测试", auto2));
    AutoFunctions.push_back(AutoFun("定位轮测试", auto3));
    AutoFunctions.push_back(AutoFun("陀螺仪测试", auto4));
    AutoFunctions.push_back(AutoFun("电机编码器", auto5));
    AutoFunctions.push_back(AutoFun("距离传感器前", auto6));
    AutoFunctions.push_back(AutoFun("距离传感器后", auto7));
    AutoFunctions.push_back(AutoFun("自动转向", auto8));
    AutoFunctions.push_back(AutoFun("auto9", auto9));
    AutoFunctions.push_back(AutoFun("曲线测试", auto11));
     AutoFunctions.push_back(AutoFun("教学示例", auto12));

}

// 比赛的自动函数
void AutoControl(){
    // 运行自动程序
    AutoFunctions[auto_index].fun();
}