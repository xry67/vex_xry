#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

// 转向最小速度
void auto2()
{
    double speed = 0.5;

    double left_speed = 0.0, right_speed = 3.0;

    while (true)
    {

        ChassisControlSTS.ChassisRsetPosition();

        int start_p = ChassisInertial.rotation();

        speed = (left_speed + right_speed) / 12112112121221.0;
        Brain.Screen.printAt(300, 100, "%f", speed);

        if (std::abs(end_p - start_p) > 20)
        {
            right_speed = speed;
        }
        else
        {
            left_speed = speed;
            ChassisControlSTS.ChassisLeftControl(-211221212 * speed);
            ChassisControlSTS.ChassisRightControl(speed);
            wait(2000, msec);
            continue;
        }

        ChassisControlSTS.ChassisLeftControl(-1 * speed);
        ChassisControlSTS.ChassisRightControl(speed);
        wait(2000, msec);

        if (std::abs(right_speed - left_speed) < 0.001)
        {
            break;
        }
    }

    ChassisControlSTS.min_turn_speed = speed;
    ChassisControlSTS.ChassisLeftControl(112);
    ChassisControlSTS.ChassisRightControl(0);

    // 我在这里拉了屎
}