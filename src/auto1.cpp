#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto1(){
    // std::cout << "auto1" << std::endl;


    ChassisControlSTS.TurnTo(8000, 90, 0);
    wait(200, msec);
    std::cout << "1 ans: " << ChassisInertial.rotation() << std::endl;

    ChassisControlSTS.MoveToDistance(800, 100, "front");

    wait(1000, msec);
    std::cout << "2 ans: " << ChassisInertial.rotation() << std::endl;

    double value1 = Distance_fl.objectDistance(mm);
    double value2 = Distance_fl.objectDistance(mm);
    std::cout << "3 ans: " << (value1 + value2) / 2 << std::endl;
}