#ifndef REMOTECONTROL_H  
#define REMOTECONTROL_H

#include "vex.h"
#include "robot-config.h"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

// 不同的遥控单元
class RemoteControlCell
{
    public:
    char* name;
    void (*fun)(bool, bool, int, std::string, double);

    RemoteControlCell(char* name_, void (*fun_)(bool, bool, int, std::string, double)){
        name = name_;
        fun  = fun_;
    }

    // 运行遥控程序
    void Work(bool SaveFile, bool ReadFile, int OutTime, std::string file_name, double timeout){
        fun(SaveFile, ReadFile, OutTime, file_name, timeout); 
    }

};


// 遥控
void RemoteControl();

void AutoRemoteControl(std::string file_name, int timeout);

void RemoteControlUnit(bool SaveFile, bool ReadFile, int OutTime, std::string file_name, double timeout);



#endif


