#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"


// 左9
void auto3(){
   digitalH.set(1);
   ChassisControlSTS.CurveMove(10, 780, 700, 'q');

    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ToolMotor2.spin(forward, 7, pct);

    ChassisControlSTS.Move(5, 1800, 'n', true);
  
    ChassisControlSTS.WaitMoveFinish();

    ChassisControlSTS.Move(5, 700, 'u');


    wait(2000, msec);

    wait(200, msec);
    digitalE.set(1);

    ChassisControlSTS.CurveMove(5, 600, 1500, 'd');

    ChassisControlSTS.Move(10, 550, 'u'); 

    digitalD.set(1);
    digitalC.set(0);

    ChassisControlSTS.TurnTo(10, 180, false); 
    ToolMotor2.spin(forward, 16, voltageUnits::volt);

    ChassisControlSTS.Move(10, 550, 'u', true); 
    wait(600, msec);
    SetToolMotorSpin(-16, 16, 16, 1800);
    AutoPush();
    ChassisControlSTS.WaitMoveFinish();

    digitalG.set(1);
    ToolMotor1.spin(forward, -16, voltageUnits::volt);

    ChassisControlSTS.Move(10, 1300, 'n'); 
    SetToolMotorSpin(-16, 16, 16, 700);
    AutoIntake();

    ChassisControlSTS.Move(16, 1320, 178, 'u', true); // 1550
    digitalD.set(1);    

    wait(200, msec);
    digitalG.set(0); 
    wait(700, msec);  
    ToolMotor2.spin(forward, 16, voltageUnits::volt);

    while(ChassisControlSTS.IsMoving()){
      if (ChassisControlSTS.GetPositions_mean() > 1200){
        break;
      }
      wait(10, msec);
    }

    SetToolMotorSpin(-16, 16, 16, 2000);
    AutoPush();

}
