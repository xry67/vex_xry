#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto1(){
    
    ChassisControlSTS.Target.SetColor(Red);
    ChassisControlSTS.TurnToTarget(2000, 90, 'r');

    wait(2000, msec);
}