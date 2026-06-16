#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "RobotControl.h"
#include "RemoteControl.h"
#include "AutoTools.h"

void auto1()
{

    // ChassisControlSTS.Turn_PID(10, 90, 0.8, 0.01, 0.05, 2);
    // ChassisControlSTS.Turn_PID(10, 180, 0.8, 0.001, 0.05, 2);
    // ChassisControlSTS.Turn_PID(10, 270, 0.8, 0.005, 0.05, 2);
    // ChassisControlSTS.Turn_PID(10, 360, 0.8, 0.0001, 0.05, 2);

    ChassisControlSTS.Move(10, 300, 0, 'n');
    ChassisControlSTS.Turn_PID(10, 180, 0.8, 0.001, 0.05, 2);
}
