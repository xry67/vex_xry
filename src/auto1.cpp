#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto1(){
    // std::cout << "auto1" << std::endl;

    ChassisControlSTS.Move(1000, 1000, 0,'n', false);
    ChassisControlSTS.Turn(800, 90, 'l');
    ChassisControlSTS.Move(1000, 1000, 0,'n', false);
    ChassisControlSTS.Move(1000, 1000, 0,'u', false);
    ChassisControlSTS.Turn(800, 90, 'r');
    ChassisControlSTS.Move(1000, 1000, 0,'u', false);
}