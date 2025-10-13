#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"

// 右6+1 队牌对位
void auto6(){
double k=1;
    ChassisInertial.setRotation(0, degrees);
    ChassisControlSTS.Over_count = 5;
    ChassisControlSTS.CurveMove(10, 780, 700, 'p');

    ToolMotor1.spin(forward, -16, voltageUnits::volt);

    ChassisControlSTS.Move(4, 570 * k, 'n'); // 1550

    ChassisControlSTS.TurnTo(14, 360  - 46, false); // 1550

    ChassisControlSTS.Move(16, 560 * k, 'n', true); // 1550
    ChassisControlSTS.WaitPositionArrive(500);
    ToolMotor2.spin(forward, -16, voltageUnits::volt);
    ToolMotor1.spin(forward, 16, voltageUnits::volt);
    wait(500, msec);
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, 0, voltageUnits::volt);
    ChassisControlSTS.WaitMoveFinish();
    digitalH.set(1);
    digitalE.set(1);

    ChassisControlSTS.Move(10, 1970, 'u'); // 1550        

    ChassisControlSTS.turn_allowable_error = 1;
    ChassisControlSTS.TurnTo(10, 180, false);

    digitalG.set(1);
    wait(200, msec);
    ChassisControlSTS.Move(14, 900, 'n', true); // 1550  
    ChassisControlSTS.WaitPositionArrive(850);
    ChassisControlSTS.StopMoving();
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ChassisControlSTS.ChassisLeftControl(2);
    ChassisControlSTS.ChassisRightControl(2);
    for (int i=0; i< 100; i++){
        if (std::abs(ToolMotor1.velocity(dps)) < 5){
            ToolMotor1.spin(forward, 16, voltageUnits::volt);
            wait(100, msec);
            i += 10;
        }
        ToolMotor1.spin(forward, -16, voltageUnits::volt);


        wait(10, msec);
    }

    wait(1000, msec);
    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);

    ChassisControlSTS.Move(7, 1500, -170, 'u', true); // 1550
    ToolMotor2.spin(forward, 16, voltageUnits::volt);
    ToolMotor3.spin(forward, 3, voltageUnits::volt);
    digitalD.set(1);  
    ChassisControlSTS.WaitPositionArrive(500);
    ChassisControlSTS.ResetStraightAngle(-180);

    digitalG.set(0);
    digitalE.set(1);

    ChassisControlSTS.WaitPositionArrive(1100);
    digitalB.set(1);

    ToolMotor2.spin(forward, 16, voltageUnits::volt);
    ToolMotor3.spin(forward, 16, voltageUnits::volt);
    ChassisControlSTS.StopMoving();
    ChassisControlSTS.ChassisLeftControl(-2);
    ChassisControlSTS.ChassisRightControl(-2);
    wait(2000, msec);

   
}