#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "RobotControl.h"
#include "RemoteControl.h"
#include "AutoTools.h"

// siu

void auto6()
{
    /*
    ChassisControlSTS.Move(9.6, 1000, 0, 'n');

    ChassisControlSTS.Turn_PID(9.6, -50);

    ToolMotor6.spin(forward, -12.8, volt);

    ChassisControlSTS.Move(9.6, 3900, -47, 'n', true); //2828

    ChassisControlSTS.WPA50(2700);

    ChassisControlSTS.ResetMaxSpeed(2.3);

    ChassisControlSTS.WaitMoveFinish();

    Claw.LiftMove(2000);

    ChassisControlSTS.Turn_PID(9.6, -165);

    ChassisControlSTS.Move(9.6, 800, -165, 'n');

    Claw.LiftMove(-2000);

    ChassisControlSTS.Move(9.6, -750, -165, 'n');
    */
    Motor1.spinFor(forward, 900, degrees);
    // digitalH.set(1);
    Claw.changeMod();

    Claw.LiftMove(0, true);

    ChassisControlSTS.Move(12.8, 650, 0, 'n');
    ChassisControlSTS.Turn_PID(12.8, 15, 0.8, 0, 0, 10);
    ChassisControlSTS.Move(6, 1900, 90, 'n', true);

    digitalH.set(0);
    ChassisControlSTS.WPA50(300);
    wait(200, msec);
    Claw.LiftMove(-2000, true);
    Claw.mod_move_flag = true;
    Claw.mod_motor.spin(forward, -8, volt);
    wait(500, msec);
    Claw.mod_move_flag = false;

    ToolMotor6.spin(forward, 1.5, volt);
    ChassisControlSTS.Move(10, 450, 90, 'u');

    // 拿第一个pincup
    ToolMotor6.spin(reverse, 12.8, volt);
    ChassisControlSTS.Turn_PID(10, 35);
    ChassisControlSTS.Move(7, 1300, 35, 'n', true);
    Claw.changeMod();
    wait(100, msec);
    Claw.changeMod();
    wait(100, msec);
    Claw.LiftMove(-2000, true);
    ChassisControlSTS.WPA50(300);
    ChassisControlSTS.ResetMaxSpeed(2);
    ChassisControlSTS.WaitMoveFinish();

    // 放第一个pincup
    Claw.LiftMove(1500, true);
    ChassisControlSTS.Turn_PID(12.8, 70, 0.8, 0, 0, 10);
    ChassisControlSTS.Move(12.8, 1500, 180, 'n', true);
    ChassisControlSTS.WPA50(350);
    ChassisControlSTS.ResetMaxSpeed(3);
    wait(400, msec);
    ToolMotor6.stop(coast);
    Claw.LiftMove(-2000, true);
    wait(300, msec);
    ChassisControlSTS.Move(10, 450, 180, 'u');

    // 拿第二个pincup
    ToolMotor6.spin(reverse, 12.8, volt);
    ChassisControlSTS.Turn_PID(10, 128);
    Claw.LiftMove(-2000, true);
    ChassisControlSTS.Move(7, 1200, 128, 'n', true);
    ChassisControlSTS.WPA50(300);
    ChassisControlSTS.ResetMaxSpeed(2);
    ChassisControlSTS.WaitMoveFinish();

    // 放第二个pincup
    Claw.LiftMove(250, true);
    ChassisControlSTS.Move(12.8, 99999, 122, 'u', true);
    ChassisControlSTS.WPA50(700);

    ChassisControlSTS.ResetStraightAngle(180);
    Claw.LiftMove(1500, true);
    ChassisControlSTS.WPA50(1000);
    ChassisControlSTS.Move(12.8, 99999, 200, 'n', true);

    ChassisControlSTS.WPA50(200);
    ChassisControlSTS.ResetStraightAngle(255);
    ChassisControlSTS.WPA50(1200);
    ChassisControlSTS.ResetStraightAngle(270);
    ChassisControlSTS.ResetMaxSpeed(3);

    ChassisControlSTS.WPA50(1500);
    wait(600, msec);
    ToolMotor6.stop(coast);
    Claw.LiftMove(-2000, true);
    wait(300, msec);
    ChassisControlSTS.Move(12.8, 1000, 270, 'u', true);
    wait(500, msec);

    return;
}