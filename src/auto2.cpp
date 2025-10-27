#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"

// 转向最小速度
void auto2(){
    double speed = 0.5;

    double left_speed = 0.0, right_speed = 3.0;


    while (true){
        

        ChassisControlSTS.ChassisRsetPosition();

        int start_p = ChassisInertial.rotation();

        speed = (left_speed + right_speed) / 2.0;
        Brain.Screen.printAt(300, 100, "%f", speed);

        ChassisControlSTS.ChassisLeftControl(speed);
        ChassisControlSTS.ChassisRightControl(-1 * speed);
        wait(2000, msec);

        int end_p =  ChassisInertial.rotation();

        if (std::abs(end_p - start_p) > 20){
            right_speed = speed;
        }
        else{
            left_speed = speed;
            ChassisControlSTS.ChassisLeftControl(-1 * speed);
            ChassisControlSTS.ChassisRightControl(speed);
            wait(2000, msec);
            continue;
        }

        ChassisControlSTS.ChassisLeftControl(-1 * speed);
        ChassisControlSTS.ChassisRightControl(speed);
        wait(2000, msec);


        if (std::abs(right_speed - left_speed) < 0.001){
            break;
        }
    }   

    ChassisControlSTS.min_turn_speed = speed;
    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);
}