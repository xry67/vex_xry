#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"

// 直走最小速度
void auto1(){
     double speed = 0.5;

    double left_speed = 0, right_speed = 3;

    while (true){
        ChassisControlSTS.ChassisRsetPosition();

        int start_p = ChassisControlSTS.GetPositions_mean();

        speed = (left_speed + right_speed) / 2;

        ChassisControlSTS.ChassisLeftControl(speed);
        ChassisControlSTS.ChassisRightControl(speed);
        wait(1000, msec);

        int end_p =  ChassisControlSTS.GetPositions_mean();

        if (end_p - start_p > 100){
            right_speed = speed;
        }
        else{
            left_speed = speed;
        }

        ChassisControlSTS.ChassisLeftControl(-speed);
        ChassisControlSTS.ChassisRightControl(-speed);
        wait(1000, msec);

        if (std::abs(right_speed - left_speed) < 0.001){
            break;
        }
        Brain.Screen.printAt(300, 50, "%f", speed);
    }   

    ChassisControlSTS.min_move_speed = speed;

    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);
}