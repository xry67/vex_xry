#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"

// 左6+1 队牌对位
void auto5(){
    double k=0.82;
    digitalH.set(1);
    digitalE.set(1);
    ChassisControlSTS.Over_count = 5;
    ChassisControlSTS.CurveMove(10, 780, 700, 'q');

    ToolMotor1.spin(forward, -16, voltageUnits::volt);

    ChassisControlSTS.Move(4, 720 * k, 'n'); // 1550
    
    // ChassisControlSTS.Turn_PPS = {0.0935, 0.0, 3};
    ChassisControlSTS.TurnTo(14, 228, false); // 1550

    ChassisControlSTS.Move_PPS = {0.01, 0, 2};
    ChassisControlSTS.Move(16, 820 * k, 'u'); // 1550
    ToolMotor2.spin(forward, 16, voltageUnits::volt);
    ToolMotor3.spin(forward, -10, voltageUnits::volt);
    wait(200, msec);
    ToolMotor2.spin(forward, 0, voltageUnits::volt);
    ToolMotor3.spin(forward, 0, voltageUnits::volt);

    ChassisControlSTS.Move(12, 2040, 'n'); // 1550        

    ChassisControlSTS.turn_allowable_error = 1;
    ChassisControlSTS.TurnTo(10, 180, false);

    digitalG.set(1);
    wait(200, msec);

    ChassisControlSTS.Move(14, 800, 'n', true); // 1550  
    ChassisControlSTS.WaitPositionArrive(300);
    ChassisControlSTS.StopMoving();
    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ChassisControlSTS.ChassisLeftControl_pct(1.7);
    ChassisControlSTS.ChassisRightControl_pct(1.7);

    for (int i=0; i< 100; i++){
        if (std::abs(ToolMotor1.velocity(dps)) < 5){
            ToolMotor1.spin(forward, 16, voltageUnits::volt);
            wait(100, msec);
            i += 10;
        }
        ToolMotor1.spin(forward, -16, voltageUnits::volt);


        wait(10, msec);
    }
    ChassisControlSTS.ChassisLeftControl_pct(0);
    ChassisControlSTS.ChassisRightControl_pct(0);

    ChassisControlSTS.Move(7, 1500, -190, 'u', true); // 1550
    ToolMotor2.spin(forward, 16, voltageUnits::volt);
    ToolMotor3.spin(forward, 3, voltageUnits::volt);
    digitalD.set(1);  
    ChassisControlSTS.WaitPositionArrive(500);
    ChassisControlSTS.ResetStraightAngle(-180);

    digitalG.set(0);
    digitalE.set(1);

    ChassisControlSTS.WaitPositionArrive(1200);
    digitalB.set(1);

    ToolMotor2.spin(forward, 16, voltageUnits::volt);
    ToolMotor3.spin(forward, 16, voltageUnits::volt);
    ChassisControlSTS.WaitMoveFinish();
    ChassisControlSTS.ChassisLeftControl(-2);
    ChassisControlSTS.ChassisRightControl(-2);
    wait(2000, msec);



}