#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"


double GetTurnTime(double p, double min_turn_speed){
    double start_time = GetNowTime();
    for (int a: {90, 90, 80, 80}){
        ChassisControlSTS.Turn_PPS = {p, 0.05, 2};
        ChassisControlSTS.min_turn_speed = min_turn_speed;
        ChassisControlSTS.Turn(16, a, 'r');
        wait(500, msec);
        ChassisControlSTS.Turn_PPS = {p, 0.05, 2};
        ChassisControlSTS.min_turn_speed = min_turn_speed;
        ChassisControlSTS.TurnTo(16, 0, false);
        wait(500, msec);
    }
    return GetNowTime() - start_time;
}


void auto3(){

   ChassisControlSTS.Over_count = 5;
   GetTurnTime(0.06, 1.5);

    // int now_angle = 0;
    // for (int a: {90, -90, 180}){
    //     ChassisControlSTS.ChassisRightControl(3);
    //     ChassisControlSTS.ChassisLeftControl(3);
    //     wait(2500, msec);
    //     ChassisControlSTS.ChassisRightControl(0);
    //     ChassisControlSTS.ChassisLeftControl(0);
    //     ChassisControlSTS.AutoPPSTurnTo(16, a);
    //     Brain.Screen.printAt(100, 200, "move_start%f", GetNowTime());

    //     wait(1500, msec);

    // }


    // int now_angle = 0;
    // for (int a: {90, 180, 270, 360}){

    //     ChassisControlSTS.AutoPPSTurnTo(16, a);
    //     Brain.Screen.printAt(100, 200, "move_start%f", GetNowTime());
    //     ChassisControlSTS.Move(10, 1000, a, 'n');
    //     wait(1500, msec);

    // }

    // for (int a: {500, 1000, 1500, 2000}){
    //     double p=0.0082, min_move_speed = 0.75, maxinum_speed_acceleration = 0.4, StraightPid_P = 3;
    //     ChassisControlSTS.Move_PPS = {p, 0.5, 2};
    //     ChassisControlSTS.min_move_speed = min_move_speed;
    //     ChassisControlSTS.maxinum_speed_acceleration = maxinum_speed_acceleration;
    //     ChassisControlSTS.StraightPid->reset(StraightPid_P, 0, 0);
    //     ChassisControlSTS.Move(10, a, 0, 'n');
    //     // ChassisControlSTS.AutoPPSTurnTo(16, 180);
    //     // ChassisControlSTS.AutoPPSTurnTo(16, 0);

    //     ChassisControlSTS.Move_PPS = {p, 0.5, 2};
    //     ChassisControlSTS.min_move_speed = min_move_speed;
    //     ChassisControlSTS.maxinum_speed_acceleration = maxinum_speed_acceleration;
    //     ChassisControlSTS.StraightPid->reset(StraightPid_P, 0, 0);
    //     ChassisControlSTS.Move(10, a, 0, 'u');
    //     wait(1500, msec);

    // }

    // for (int i=0; i<1000; i++){
    //     Brain.Screen.printAt(100, 150, "////%f////", RunningAutoFun->GetAutoRunTime());
    //     wait(10, msec);
    // }

    // ChassisControlSTS.Over_count = 5;
    // ChassisControlSTS.Move(10, 3500, 0, 'n', true);
    // ChassisControlSTS.WaitPositionArrive(1200);
    // ChassisControlSTS.ResetMaxSpeed(5);
    
    // ChassisControlSTS.WaitPositionArrive(1000);
    // ChassisControlSTS.ResetMaxSpeed(2);
    // ChassisControlSTS.ResetStraightAngle(90);
    
    // ChassisControlSTS.WaitPositionArrive(1800);
    // ChassisControlSTS.ResetMaxSpeed(10);

    // ChassisControlSTS.WaitPositionArrive(3500);
    // ChassisControlSTS.Turn_PPS = {0.069, 0.0, 0};
    // ChassisControlSTS.min_turn_speed = 0.49;
    // ChassisControlSTS.TurnTo(16, 0, 0);
    
}
