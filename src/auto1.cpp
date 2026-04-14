#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "RobotControl.h"
#include "RemoteControl.h"
#include "AutoTools.h"

void auto1()
{

    // 电机检查
    for (int i = 0; i < 6; i++)
    {
        Controller.Screen.print("%s电机测试中", MotorNames[i]);
        Motors[i].spin(forward, 3, volt);

        double total_c = 0;

        for (int j = 0; j < 200; j++)
        {
            total_c += Motors[i].velocity(pct);
            wait(10, msec);
        }
        Motors[i].spin(forward, 0, percent);

        Brain.Screen.printAt(100, i * 20 + 20, "%f", total_c);
    }
}
