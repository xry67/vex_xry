#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto7(){

    ChassisControlSTS.Move(10, 2000, 'n', true);
    ChassisControlSTS.WaitPositionArrive(1000);
    ChassisControlSTS.StopMoving();

    ChassisControlSTS.ChassisLeftControl(-5);
    ChassisControlSTS.ChassisRightControl(-5);
    ToolMotor1.spin(forward, 100, pct);
    ToolMotor2.spin(forward, 100, pct);
    ToolMotor3.spin(forward, 100, pct);
    wait(1000, msec);

    
    ChassisControlSTS.Turn(10, 90, 'l');
    ToolMotor1.spin(forward, 100, pct);

}