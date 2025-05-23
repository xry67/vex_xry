#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto2(){
    AiTarget Target = AiTarget(Red);
    // Target.Limit_x = {0, aiVision_w/2};
    ChassisControlSTS.FollowTarget(1000, 3000, Target, true);
    // wait(8000, msec);
    Brain.Screen.printAt( 10, 150, "start");
    while(true){
        if (Target.Target_CenterY > 170){
            ChassisControlSTS.StopMoving();
            ChassisControlSTS.Turn(8000, 180, 'l');
            
            break;
        }
        Brain.Screen.printAt( 10, 150, "h %d" , Target.Target_CenterY);
        wait(10, msec);
    }
    wait(2000, msec);
}