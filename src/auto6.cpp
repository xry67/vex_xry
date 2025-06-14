#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto6(){
    for (int l=1500; l>100; l-=500){
        ChassisControlSTS.MoveToDistance(7, l, "front");
        ChassisControlSTS.MoveToDistance(7, 1500, "front");
    }
    ChassisControlSTS.MoveToDistance(7, 200, "front");
    ChassisControlSTS.MoveToDistance(7, 1500, "front");
}