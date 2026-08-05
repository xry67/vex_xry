#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "RobotControl.h"
#include "RemoteControl.h"
#include "AutoTools.h"

void auto5()
{
    
    ChassisControlSTS.Turn_PID(10, 90, 0.8, 0.035, 0.05, 2);
}
