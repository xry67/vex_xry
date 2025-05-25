#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto2(){
    ChassisControlSTS.Target.SetColor(Red);
    // ChassisControlSTS.Target.Align = 0.5;
    ChassisControlSTS.FollowTarget(1000, 3000, true);
    int i=0;
    while(ChassisControlSTS.IsMoving()){
        if (ChassisControlSTS.Target.Target_CenterY > 200){
            ChassisControlSTS.StopMoving();
            ChassisControlSTS.SetStopMod("c");
            ChassisControlSTS.MoveBySmallWheel(2000, 100, 0, 'u');
            
            break;
        }
        i += 1;
        wait(10, msec);
    }
    ChassisControlSTS.StopMoving();
    Brain.Screen.clearScreen();
}