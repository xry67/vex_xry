#include "vex.h"
#include "robot-config.h"

void auto2(){
    std::cout << "auto2" << std::endl;
    motor Motor = motor(PORT12, ratio6_1, 0);//11
    
    Motor.resetPosition();
    Motor.spinFor(forward, 100, degrees, 50, dps, true);
    
    std::cout << Motor.position(degrees) << std::endl;


}