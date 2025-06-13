#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto2(){
    ChassisControlSTS.Target.SetColor(Red);
    // ChassisControlSTS.Target.Limit_x = {0, aiVision_w / 2};
    ChassisControlSTS.FollowTarget(7, 3000, true);

    while(ChassisControlSTS.IsMoving()){
        if (ChassisControlSTS.Target.Target_CenterY > 180){
            ChassisControlSTS.StopMoving();
            ChassisControlSTS.SetStopMod("c");
            ChassisControlSTS.MoveBySmallWheel(7, 100, 0, 'u');
            
            break;
        }
        wait(10, msec);
    }
    ChassisControlSTS.StopMoving();
    Brain.Screen.clearScreen();

    wait(2000, msec);
}