#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto1(){
    ToolMotorSpin(2000, 0, 0, 3000, true);
    ChassisControlSTS.Move(16, 2000, 0.0, 'n');
    
}