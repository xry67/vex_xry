#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "RobotControl.h"
#include "RemoteControl.h"
#include "AutoTools.h"

void auto1()
{

    // ChassisControlSTS.Move(8, 1075, 0, 'n', 0);
    // ChassisControlSTS.Turn_PID(10, -90, 0.8, 0.035, 0.05, 2);

    // ChassisControlSTS.Move(5, 600, -90, 'n', 1);
    // Claw.LiftMove(StartLiftPosititon + 15);
    // ChassisControlSTS.WaitMoveFinish();
    // Claw.LiftMove(StartLiftPosititon - 15);
    // ChassisControlSTS.Move(5, 700, -90, 'u', 0);
    // ChassisControlSTS.Turn_PID(10, -75, 0.8, 0.035, 0.05, 2);
    // ChassisControlSTS.Move(8, 1075, 0, 'q', 0);

    ToolMotor3.spin(forward, 8, volt);
    ChassisControlSTS.Move(8, 300, 0, 'n');
    ToolMotor3.spin(forward, 0, volt);

    ChassisControlSTS.Turn_PID(10, -30, 0.8, 0.035, 0.05, 2);


    
}
