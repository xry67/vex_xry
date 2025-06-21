#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

// 红方低柱
void auto9(){
    ChassisControlSTS.Over_count = 2;
    double speed = 100;
    ChassisControlSTS.MoveToDistance(speed, 400, "front");
   
    ChassisControlSTS.TurnTo(speed, 90, false);

    ChassisControlSTS.SetStopModCoast();
    ChassisControlSTS.MoveBySmallWheel(speed, 400, 0, 'n');
    // ChassisControlSTS.TurnTo(speed, 90, false);

    // ChassisControlSTS.StopMoving();
    // ChassisControlSTS.MoveBySmallWheel(500, 350, 0, 'n');

    ChassisControlSTS.SetStopModHold();
    ChassisControlSTS.MoveBySmallWheel(speed, 1350, 0, 'u');
    ChassisControlSTS.MoveBySmallWheel(speed, 850, 0, 'n');

    ChassisControlSTS.Target.SetColor(Red);
    // ChassisControlSTS.TurnToTarget(2000, 115, 'r');
    ChassisControlSTS.TurnTo(speed, 225, false);
    ChassisControlSTS.MoveBySmallWheel(speed, 1850, 0, 'n');


    ChassisControlSTS.ChassisStoppingCoast();
}