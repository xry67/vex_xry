#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto4(){

    ChassisControlSTS.Target.SetColor(Red);
    ChassisControlSTS.Target.Limit_y = {aiVision_h / 3, aiVision_h};

    ChassisControlSTS.FollowTarget(1000, 3000, true);
    while(ChassisControlSTS.IsMoving()){
        std::cout << "Target_CenterY" << ChassisControlSTS.Target.Target_CenterY  << std::endl;
        if (ChassisControlSTS.Target.Target_CenterY > 160){
            ChassisControlSTS.StopMoving();
            ChassisControlSTS.MoveBySmallWheel(2000, 100, 0, 'u');
            
            break;
        }
        wait(10, msec);
    }

    ChassisControlSTS.FollowTarget(1000, 3000, true);
    while(ChassisControlSTS.IsMoving()){
        std::cout << "Target_CenterY" << ChassisControlSTS.Target.Target_CenterY  << std::endl;
        if (ChassisControlSTS.Target.Target_CenterY > 160){
            ChassisControlSTS.StopMoving();
            ChassisControlSTS.MoveBySmallWheel(2000, 100, 0, 'u');
            
            break;
        }
        wait(10, msec);
    }

    ChassisControlSTS.FollowTarget(1000, 3000, true);
    while(ChassisControlSTS.IsMoving()){
        if (ChassisControlSTS.Target.Target_CenterY > 180){
            ChassisControlSTS.StopMoving();
            ChassisControlSTS.MoveBySmallWheel(2000, 100, 0, 'u');
            
            break;
        }
        wait(10, msec);
    }


    ChassisControlSTS.SetStopModCoast();


    wait(2000, msec);



}