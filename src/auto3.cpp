#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "RobotControl.h"
#include "RemoteControl.h"
#include "AutoTools.h"


void UniformMotion(double speed, double angle, double time){
    ChassisControlSTS.ChassisForwardAngle(angle); // 设置匀速运动的角度

    for (int i=0; i<time/10; i++){
        ChassisControlSTS.ChassisForward(speed);  // 匀速运动
        wait(10, msec);
    }   

}

void auto3()
{
    double i = 0.06;
    while (1){
        if (Controller.ButtonA.pressing()){
            ChassisControlSTS.Turn_PID(8, 90, 0.4, i, 0.03, 2);
            Controller.rumble(".");
        }
        if (Controller.ButtonB.pressing()){
            ChassisControlSTS.Turn_PID(8, 2.3, 0.4, i, 0.03, 2);
            Controller.rumble(".");
        }
        if (Controller.ButtonX.pressing()){
            ChassisInertial.setRotation(0, deg);
            Controller.rumble(".");
        }
        
        wait(10, msec);
    }
}