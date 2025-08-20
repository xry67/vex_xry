#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"

// 左6+1
void auto5(){
    double k=1;

    ChassisControlSTS.Over_count = 4;
    ChassisControlSTS.Move(16, 600 * k, 0.0, 'n'); // 1550

    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    ChassisControlSTS.Turn(10, 44, 'l');

    ChassisControlSTS.Move(6, 300 * k, 0.0, 'n'); // 1550
    ChassisControlSTS.Move(2, 640 * k, 0, 'n'); // 1550
    
    ChassisControlSTS.Turn_PPS = {0.0935, 0.0, 3};
    ChassisControlSTS.TurnTo(14, 222, false); // 1550

    ChassisControlSTS.Move_PPS = {0.01, 0, 2};
    ChassisControlSTS.Move(16, 820 * k, 0.0, 'u', true); // 1550
    wait(800, msec);
   ToolMotor1.spin(forward, -14, voltageUnits::volt);
    ToolMotor2.spin(forward, 16, voltageUnits::volt);
    ToolMotor3.spin(forward, -16, voltageUnits::volt);
    wait(500, msec);
    // ToolMotor1.spin(forward, 0, voltageUnits::volt);
    ToolMotor2.spin(forward, 0, voltageUnits::volt);
    ToolMotor3.spin(forward, 0, voltageUnits::volt);
    ChassisControlSTS.WaitMoveFinish();

    ChassisControlSTS.Over_count = 12;
    ChassisControlSTS.Move(14, 300 * k, 0.0, 'n'); // 1550
    ChassisControlSTS.Turn_PPS = {0.3, 0.0, 3};
    ChassisControlSTS.TurnTo(14, 230, false); // 1550
    ChassisControlSTS.Move(14, 2420 * k, 0.0, 'n'); // 1550

    
    ChassisControlSTS.Over_count = 15;
    ChassisControlSTS.TurnTo(8, 179, false);
    digitalG.set(1);
    ChassisControlSTS.Over_count = 4;
    digitalE.set(1);

    ToolMotor1.spin(forward, -16, voltageUnits::volt);
    // ChassisControlSTS.ComboMove(    {ComboMoveParams(5, 600 * k, 0.0, 'n', nullptr)}   );
    ChassisControlSTS.Move(14, 700, 0.0, 'n'); // 1550

    SetToolMotorSpin(-16, 16, 16, 1500);
    AutoIntake();
    
    ChassisControlSTS.ComboMove(    {ComboMoveParams(5, 200, 0.0, 'u', nullptr)}   );

    ChassisControlSTS.TurnTo(8, 179, false);
    ChassisControlSTS.Move(16, 1320 * k, 0, 'u', true); // 1550
    digitalD.set(1);    

    wait(500, msec);
    digitalG.set(0);
    digitalD.set(1);    
    wait(700, msec);

    SetToolMotorSpin(-16, 16, 16, 2000);
    AutoPush();
    
    ChassisControlSTS.WaitMoveFinish();
}