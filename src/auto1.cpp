#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto1(){
    
    ChassisControlSTS.MoveToDistance(8000, 400, "front");
   
    ChassisControlSTS.TurnTo(2000, 90, false);

    ChassisControlSTS.SetStopModCoast();
    ChassisControlSTS.MoveBySmallWheel(1000, 500, 0, 'n', true);
    wait(500, msec);
    while (ChassisControlSTS.IsMoving()){
        wait(10, msec);
    }
    ChassisControlSTS.TurnTo(2000, 90, false);

    // ChassisControlSTS.StopMoving();
    // ChassisControlSTS.MoveBySmallWheel(500, 350, 0, 'n');

    ChassisControlSTS.SetStopModHold();
    ChassisControlSTS.MoveBySmallWheel(2000, 1350, 0, 'u');
    ChassisControlSTS.MoveBySmallWheel(1000, 850, 0, 'n');

    ChassisControlSTS.Target.SetColor(Red);
    // ChassisControlSTS.TurnToTarget(2000, 115, 'r');
    ChassisControlSTS.TurnTo(2000, 225, false);
    ChassisControlSTS.MoveBySmallWheel(1000, 1850, 0, 'n');


    ChassisControlSTS.ChassisStoppingCoast();
}