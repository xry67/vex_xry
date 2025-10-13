#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"

// 右9
void auto4(){

    // ChassisControlSTS.Move(16, 180, 'n'); 
    ChassisControlSTS.CurveMove(10, 780, 700, 'p');
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, 3, voltageUnits::volt);

    ChassisControlSTS.Move(1.5, 800 + 700 + 370, 'n', true);
    ChassisControlSTS.WaitPositionArrive(500);
    ChassisControlSTS.ResetMaxSpeed(16);
    ChassisControlSTS.WaitPositionArrive(1200);
    ChassisControlSTS.ResetMaxSpeed(1.7);
    ChassisControlSTS.WaitMoveFinish();
    digitalG.set(1);
    // // wait(1000, msec);
    digitalE.set(1);

    ChassisControlSTS.Move_PPS = {0.030, 0.000, 2};
    ChassisControlSTS.CurveMove(5, 600, 1500, 'b');

    ChassisControlSTS.Move(16, 700, 'u'); 

    digitalD.set(1);
    digitalC.set(0);

    ChassisControlSTS.TurnTo(10, 180, false); 

    ChassisControlSTS.Move(16, 650, 'u', true);
    wait(300, msec);
 
    wait(100, msec);
    SetToolMotorSpin(-16, 16, 16, 2000);
    AutoPush();
    ChassisControlSTS.WaitMoveFinish();

    digitalG.set(1);
    ToolMotor1.spin(forward, -16, voltageUnits::volt);

    ChassisControlSTS.Move(16, 1400, 'n'); 
    SetToolMotorSpin(-16, 2, 16, 1500);
    AutoIntake();
    ToolMotor2.spin(forward, 3, voltageUnits::volt);

    ChassisControlSTS.Move(16, 1520, -185, 'u', true); // 1550
    digitalD.set(1);    

    wait(200, msec);
    digitalG.set(0); 

    ChassisControlSTS.WaitPositionArrive(1000);

    SetToolMotorSpin(-16, 16, 16, 1800);
    AutoPush();

}