#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"


void auto8(){

    ChassisControlSTS.Move(1, 2000, 'n');
}