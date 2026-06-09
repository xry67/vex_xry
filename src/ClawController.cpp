#include "vex.h"
using namespace vex;

double moveToConNotMove(vex::motor motor_, double speed, int max_count){
    int count = 0;
    while (true)
    {
        motor_.spin(forward, speed, volt);
        // std::cout << "-->" << reversal_motor.velocity(dps) << std::endl;
        if (abs(motor_.velocity(dps)) <= 2){
            count += 1;
        }else{
            count = 0;
        }
        if (count >= max_count){
            break;
        }
        wait(10, msec);
    }
    motor_.spin(forward, 0, volt);
    return motor_.position(deg);
}