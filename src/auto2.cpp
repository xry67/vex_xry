#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto2(){
    AiTarget Target = AiTarget(Red);
    // Target.Limit_x = {0, aiVision_w/2};
    ChassisControlSTS.FollowTarget(1000, 6000, Target, true);



}