#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto1(){
    
    ChassisControlSTS.MoveToDistance(7, 400, "front");
   
    ChassisControlSTS.TurnTo(7, 90, false);

    ChassisControlSTS.SetStopModCoast();
    ChassisControlSTS.MoveBySmallWheel(7, 500, 0, 'n');
    ChassisControlSTS.TurnTo(7, 90, false);

    // ChassisControlSTS.StopMoving();
    // ChassisControlSTS.MoveBySmallWheel(500, 350, 0, 'n');

    ChassisControlSTS.SetStopModHold();
    ChassisControlSTS.MoveBySmallWheel(7, 1350, 0, 'u');
    ChassisControlSTS.MoveBySmallWheel(7, 850, 0, 'n');

    ChassisControlSTS.Target.SetColor(Red);
    // ChassisControlSTS.TurnToTarget(2000, 115, 'r');
    ChassisControlSTS.TurnTo(7, 225, false);
    ChassisControlSTS.MoveBySmallWheel(7, 1850, 0, 'n');


    ChassisControlSTS.ChassisStoppingCoast();
}