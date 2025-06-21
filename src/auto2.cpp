#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"


// 相机测试
void auto2(){
    ChassisControlSTS.Target.SetColor(Red);
    ChassisControlSTS.Target.Limit_y = {aiVision_h / 2, aiVision_h};
    ChassisControlSTS.FollowTarget(5, 3000, true);

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

    wait(2000, msec);
}