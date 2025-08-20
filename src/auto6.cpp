#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"

// 右6+1
void auto6(){
    double k=1;

    ChassisControlSTS.Over_count = 4;
    ChassisControlSTS.Move(16, 600 * k, 0.0, 'n'); // 1550

    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ChassisControlSTS.Turn(10, 44, 'r');

    ChassisControlSTS.Move(6, 450 * k, 0.0, 'n'); // 1550
    ChassisControlSTS.Move(2, 690 * k, 0, 'n'); // 1550
    
    ChassisControlSTS.Turn_PPS = {0.0935, 0.0, 3};
    ChassisControlSTS.TurnTo(14, 315, false); // 1550

    ChassisControlSTS.Move_PPS = {0.01, 0, 2};
    ChassisControlSTS.Move(16, 480 * k, 0.0, 'n', true); // 1550
    digitalE.set(1);

    wait(800, msec);
    ToolMotor1.spin(forward, 14, voltageUnits::volt);

   ToolMotor2.spin(forward, -14, voltageUnits::volt);

    wait(900, msec);
    ToolMotor1.spin(forward, -14, voltageUnits::volt);
    ToolMotor2.spin(forward, 0, voltageUnits::volt);

    ChassisControlSTS.WaitMoveFinish();

    ChassisControlSTS.Over_count = 12;

    ChassisControlSTS.TurnTo(14, 310, false); // 1550
    ChassisControlSTS.Move(14, 2350 * k, 0.0, 'u'); // 1550

    
    ChassisControlSTS.Over_count = 15;
        digitalG.set(1);

    ChassisControlSTS.TurnTo(8, 179, false);
    ChassisControlSTS.Over_count = 4;
    digitalE.set(1);

    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    // ChassisControlSTS.ComboMove(    {ComboMoveParams(5, 600 * k, 0.0, 'n', nullptr)}   );
    ChassisControlSTS.Move(14, 850, 0.0, 'n'); // 1550

    SetToolMotorSpin(-16, 16, 16, 1700);
    AutoIntake();
    ChassisControlSTS.ComboMove(    {ComboMoveParams(5, 200, 0.0, 'u', nullptr)}   );
    ChassisControlSTS.TurnTo(8, 179, false);
    ChassisControlSTS.Move(16, 1320 * k, 0, 'u', true); // 1550
    digitalD.set(1);    

    wait(200, msec);
    digitalG.set(0);   
    wait(400, msec);
       ToolMotor2.spin(forward, 16, voltageUnits::volt);

    SetToolMotorSpin(-16, 16, 16, 2000);
    AutoPush();
    
    ChassisControlSTS.WaitMoveFinish();
   
}