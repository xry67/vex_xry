#ifndef REMOTECONTROL_H  
#define REMOTECONTROL_H

#include "vex.h"
#include "robot-config.h"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

// 其他控制
void RemoteControl();

void AutoRemoteControl(std::string file_name, int timeout);

void RemoteControlUnit(bool SaveFile, bool ReadFile, int OutTime, std::string file_name, double timeout);



#endif


