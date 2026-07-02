#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "RobotControl.h"
#include "RemoteControl.h"
#include "AutoTools.h"

void auto1()
{

    ChassisControlSTS.Move(4, 500, 0, 'n', 1);
    ChassisControlSTS.WaitPositionArrive(500);
    ChassisControlSTS.ReSetToCurveMove_Angle(220, 90, 'n');
    ChassisControlSTS.WaitMoveFinish();
}
