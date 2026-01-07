#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto14(){
    
    ChassisControlSTS.Move(16, 99, 'n');
    ChassisControlSTS.Turn(16, 90, 'l');
    ChassisControlSTS.Move(16, 99, 'n');
    ChassisControlSTS.TurnTo(16, -180, 0);
    ChassisControlSTS.Move(16, 99, 'n');

}