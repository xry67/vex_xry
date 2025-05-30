#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto1(){
    
    ChassisControlSTS.MoveToDistance(8000, 370, "front");
   
    ChassisControlSTS.TurnTo(2000, 90, false);
    ChassisControlSTS.MoveBySmallWheel(2000, 350, 0, 'n');
    ChassisControlSTS.MoveBySmallWheel(2000, 1380, 0, 'u');

    ChassisControlSTS.ChassisStoppingCoast();
}