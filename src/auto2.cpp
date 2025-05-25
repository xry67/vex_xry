#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto2(){
    ChassisControlSTS.Target.SetColor(Red);
    ChassisControlSTS.FollowTarget(100, 3000, true);
    // wait(8000, msec);
    Brain.Screen.printAt( 10, 150, "start");
    while(ChassisControlSTS.IsMoving()){
        if (ChassisControlSTS.Target.Target_CenterY > 170){
            ChassisControlSTS.StopMoving();
            ChassisControlSTS.Turn(500, 180, 'l');
            
            break;
        }
        Brain.Screen.printAt( 10, 150, "h %d" , ChassisControlSTS.Target.Target_CenterY);
        wait(10, msec);
    }
    Brain.Screen.printAt( 10, 150, "end");
    ChassisControlSTS.StopMoving();
    wait(2000, msec);
}