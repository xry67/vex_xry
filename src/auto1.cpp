#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto1(){
    // std::cout << "auto1" << std::endl;


    ChassisControlSTS.Turn(800, 90, 'l');
    wait(100, msec);
    ChassisControlSTS.Turn(800, 90, 'r');
    wait(100, msec);
    ChassisControlSTS.Turn(800, 180, 'r');
    wait(100, msec);
    ChassisControlSTS.Turn(800, 90, 'r');
    wait(100, msec);
    ChassisControlSTS.Turn(800, 360, 'l');
    wait(100, msec);
    ChassisControlSTS.TurnTo(800, 0, false);
    wait(100, msec);
}